"""Fit static box terrain to an exported G1 reference using the existing USD-bound convention.

Run with the Isaac Lab interpreter as ``python -m data_generator.mixamo.fit_terrain``.
Like the CLAW passage fitter, this preserves every motion sample. Fitted geometry
is a new reconstruction, not terrain supplied by Mixamo or evidence of tracking.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from data_generator.shared.fit_overhead import body_geometry_corners
from data_generator.shared.fit_stairs import clear_front_edge, support_intervals
from isaaclab_assets import UNITREE_G1_29DOF_BALL_HAND_CFG

CLEARANCE = 0.004


class TerrainFit:
    """Measure in a path-aligned frame; export boxes in the unchanged motion frame."""

    def __init__(self, data, kind):
        self.data = data
        names = data["body_names"].tolist()
        pelvis = data["body_pos_w"][:, names.index("pelvis")]
        direction = pelvis[-1, :2] - pelvis[0, :2]
        if np.linalg.norm(direction) < 0.12:
            direction = Rotation.from_quat(data["body_quat_w"][0, names.index("pelvis")]).apply([1, 0, 0])[:2]
        if kind == "stairs" and pelvis[-1, 2] < pelvis[0, 2]:
            direction *= -1  # Stairs ascend in fitting +X, even for a descending motion.
        self.rotation = Rotation.from_euler("z", np.arctan2(direction[1], direction[0]))
        self.inverse = self.rotation.inv()
        local = body_geometry_corners(names, UNITREE_G1_29DOF_BALL_HAND_CFG.spawn.usd_path)
        matrices = (
            Rotation.from_quat(data["body_quat_w"].reshape(-1, 4))
            .as_matrix()
            .reshape(*data["body_quat_w"].shape[:2], 3, 3)
        )
        corners = np.einsum("tbij,bcj->tbci", matrices, local) + data["body_pos_w"][:, :, None, :]
        self.points = self.transform(corners)
        self.low, self.high = self.points.min(axis=2), self.points.max(axis=2)
        self.feet = self.transform(data["foot_box_corners_w"])
        self.hands = self.transform(data["hand_sphere_centers_w"])
        self.radii = data["hand_sphere_radii"]
        self.pelvis = self.transform(pelvis)
        self.names = names
        self.ground = float(min(0, self.low[..., 2].min()) - CLEARANCE)
        self.boxes, self.quats, self.roles = [], [], []
        self.details = {}
        self.bounds = np.stack([self.low.min(axis=(0, 1)), self.high.max(axis=(0, 1))])

    def transform(self, values):
        return self.inverse.apply(values.reshape(-1, 3)).reshape(values.shape)

    def add(self, center, dimensions, role, rotation=None):
        dimensions = np.asarray(dimensions, dtype=float)
        if not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError(f"Invalid box dimensions: {role}: {dimensions}")
        self.boxes.append(np.r_[self.rotation.apply(center), dimensions])
        self.quats.append((self.rotation * (rotation or Rotation.identity())).as_quat())
        self.roles.append(role)

    def top_limit(self, x0, x1, y0, y1):
        """Highest horizontal top below every overlapping swept body bound."""
        overlap = (self.high[..., 0] >= x0) & (self.low[..., 0] <= x1)
        overlap &= (self.high[..., 1] >= y0) & (self.low[..., 1] <= y1)
        return float(self.low[..., 2][overlap].min() - CLEARANCE) if overlap.any() else np.inf

    def support_box(self, rectangle, height, role, bottom=None):
        x0, x1, y0, y1 = rectangle
        height = float(min(height, self.top_limit(*rectangle)))
        bottom = min(self.ground - 0.15, height - 0.08) if bottom is None else bottom
        self.add([(x0 + x1) / 2, (y0 + y1) / 2, (height + bottom) / 2], [x1 - x0, y1 - y0, height - bottom], role)
        return height

    def floor(self):
        lo, hi = self.bounds
        self.add(
            [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, self.ground - 0.1],
            [hi[0] - lo[0] + 0.8, hi[1] - lo[1] + 0.8, 0.2],
            "ground_slab",
        )

    def passage(self):
        # Same roof/side-post construction as shared/fit_overhead.py, with no
        # artificial one-meter minimum for short Mixamo locomotion cycles.
        lo, hi = self.bounds
        a, b = np.quantile(self.pelvis[:, 0], [0.4, 0.65])
        if b - a < 0.2:
            mid = float(np.median(self.pelvis[:, 0]))
            a, b = mid - 0.1, mid + 0.1
        overlap = (self.high[..., 0] >= a) & (self.low[..., 0] <= b)
        roof = float(self.high[..., 2][overlap].max() + 0.04)
        yl, yh = lo[1] - 0.15, hi[1] + 0.15
        self.add([(a + b) / 2, (yl + yh) / 2, roof + 0.03], [b - a, yh - yl + 0.06, 0.06], "passage_roof")
        for x in (a, b):
            for y in (yl, yh):
                self.add([x, y, (self.ground + roof) / 2], [0.06, 0.06, roof - self.ground], "passage_post")
        self.details.update(roof_underside_world_z_m=roof, roof_to_swept_body_bound_clearance_m=0.04)

    def stairs(self):
        threshold = 0.3
        intervals = support_intervals(self.feet, float(self.data["fps"]), threshold)
        if len(intervals) < 2:
            threshold = 0.6
            intervals = support_intervals(self.feet, float(self.data["fps"]), threshold)
        yl, yh = self.feet[..., 1].min() - 0.2, self.feet[..., 1].max() + 0.2
        end = self.feet[..., 0].max() + 0.3
        heights = sorted(set(round(i["min_z"] - CLEARANCE, 3) for i in intervals))
        accepted = []
        for h in heights:
            if h < self.ground + 0.025 or any(abs(h - prev) < 0.035 for prev in accepted):
                continue
            # Reuse the existing exact-foot-edge clearance calculation. Place the
            # tread behind all foot edges that ever pass below its top plane.
            front = clear_front_edge(self.feet, h)
            if front >= end - 0.08:
                continue
            actual = self.support_box([front, end, yl, yh], h, "stair_tread")
            if actual > self.ground + 0.02:
                accepted.append(actual)
        if not accepted:
            raise ValueError("No elevated collision-free stair tread fits this reference")
        self.details.update(
            estimated_foot_support_intervals=intervals,
            fitted_stair_top_heights_m=accepted,
            support_speed_threshold_m_s=threshold,
        )

    def ladder(self):
        intervals = support_intervals(self.feet, float(self.data["fps"]), 0.35)
        feet = self.feet.mean(axis=2)
        targets = [self.feet[i["start"] : i["stop"], i["foot"]].mean(axis=(0, 1)) for i in intervals]
        # Foot targets are at the sole bottom rather than the ankle or box center.
        for point, interval in zip(targets, intervals):
            point[2] = interval["min_z"]
        hands = self.hands.copy()
        hands[..., 2] -= self.radii
        speed = np.linalg.norm(np.gradient(hands, 1 / float(self.data["fps"]), axis=0), axis=-1)
        for side in range(2):
            idx = np.flatnonzero(speed[:, side] <= np.quantile(speed[:, side], 0.25))
            targets.extend(hands[idx[:: max(1, len(idx) // 4)], side])
        if len(targets) < 2:
            raise ValueError("Insufficient ladder support samples")
        targets = np.asarray(targets)
        a, b = np.polyfit(targets[:, 2], targets[:, 0], 1)
        a = float(np.clip(a, -1, 1))
        b = float(np.median(targets[:, 0] - a * targets[:, 2]))
        # Measured support heights are retained; intermediate rungs bridge long
        # spacing, with the 0.20 m fallback used only if the clip has one level.
        levels = []
        for h in sorted(targets[:, 2]):
            if h > self.ground + 0.08 and (not levels or h - levels[-1] > 0.10):
                levels.append(float(h))
        spacing = float(np.clip(np.median(np.diff(levels)) if len(levels) > 1 else 0.20, 0.12, 0.28))
        for h in np.arange(self.ground + spacing, max(levels) + 0.01, spacing):
            if min(abs(h - v) for v in levels) > 0.09:
                levels.append(float(h))
        yl = min(feet[..., 1].min(), hands[..., 1].min()) - 0.18
        yh = max(feet[..., 1].max(), hands[..., 1].max()) + 0.18
        rung_points = []
        shifts = []
        for h in sorted(levels):
            desired = a * h + b
            # Put each static rung just outside the swept body bounds at its
            # height. Side rails connect the fitted rungs; they stay outside
            # the swept hand/foot width. This permits a nonuniform climb frame.
            overlap = (self.high[..., 2] >= h - 0.05 - CLEARANCE) & (self.low[..., 2] <= h + CLEARANCE)
            overlap &= (self.high[..., 1] >= yl) & (self.low[..., 1] <= yh)
            forbidden = sorted(
                zip(self.low[..., 0][overlap] - 0.06 - CLEARANCE, self.high[..., 0][overlap] + 0.06 + CLEARANCE)
            )
            merged = []
            for left, right in forbidden:
                if merged and left <= merged[-1][1]:
                    merged[-1][1] = max(merged[-1][1], right)
                else:
                    merged.append([float(left), float(right)])
            x = desired
            for left, right in merged:
                if left < x < right:
                    x = left if x - left < right - x else right
                    break
            self.add([x, (yl + yh) / 2, h - 0.025], [0.12, yh - yl, 0.05], "ladder_rung")
            rung_points.append([x, h - 0.025])
            shifts.append(float(x - desired))
        ends = [[rung_points[0][0], self.ground], *rung_points, [rung_points[-1][0], max(levels) + 0.2]]
        for p0, p1 in zip(ends[:-1], ends[1:]):
            dx, dz = np.asarray(p1) - p0
            rotation = Rotation.from_euler("y", np.arctan2(dx, dz))
            for y in (yl, yh):
                self.add(
                    [(p0[0] + p1[0]) / 2, y, (p0[1] + p1[1]) / 2],
                    [0.06, 0.06, float(np.hypot(dx, dz))],
                    "ladder_rail",
                    rotation,
                )
        self.details["rung_x_adjustment_for_body_clearance_m"] = shifts
        self.details["ladder_geometry"] = "nonuniform rungs and connected side rails fitted to the fixed reference"
        self.details.update(
            ladder_x_from_z=[a, b], estimated_support_points=targets.tolist(), rung_top_heights_m=sorted(levels)
        )

    def wall(self):
        lo, hi = self.bounds
        face = float(hi[0] + CLEARANCE)
        top = float(max(self.hands[..., 2].max() + 0.2, hi[2]))
        self.add(
            [face + 0.06, (lo[1] + hi[1]) / 2, (self.ground + top) / 2],
            [0.12, hi[1] - lo[1] + 0.5, top - self.ground],
            "climbing_wall",
        )
        self.details.update(
            wall_face_path_frame_x_m=face,
            hand_pad_to_wall_minimum_gap_m=float(np.min(face - self.hands[..., 0] - self.radii)),
        )

    def contact_box(self, kind):
        if kind == "seat":
            idx = np.argsort(self.pelvis[:, 2])[: max(3, len(self.pelvis) // 5)]
            idx = idx[:: max(1, len(idx) // 8)]
            pelvis_corners = self.points[idx, self.names.index("pelvis")]
            lower = np.argsort(pelvis_corners[..., 2], axis=1)[:, :4]
            points = np.take_along_axis(pelvis_corners, lower[..., None], axis=1).reshape(-1, 3)
        elif kind == "climb":
            intervals = support_intervals(self.feet, float(self.data["fps"]), 0.4)
            elevated = [i for i in intervals if i["min_z"] > self.ground + 0.12]
            points = []
            for i in elevated:
                point = self.feet[i["start"] : i["stop"], i["foot"]].mean(axis=(0, 1))
                point[2] = i["min_z"]
                points.append(point)
            if not points:
                return self.contact_box("vault")
            points = np.asarray(points)
        else:
            start, end = int(len(self.hands) * 0.2), max(2, int(len(self.hands) * 0.8))
            points = self.hands[start : end : max(1, (end - start) // 20)].reshape(-1, 3).copy()
            points[:, 2] -= float(np.mean(self.radii))
        best = None
        for point in points:
            for depth in (0.16, 0.28, 0.42):
                for width in (0.24, 0.40, 0.60):
                    for offset_x in (-0.35, 0, 0.35):
                        for offset_y in (-0.35, 0, 0.35):
                            x, y = point[:2] + np.array([offset_x * depth, offset_y * width])
                            rect = [x - depth / 2, x + depth / 2, y - width / 2, y + width / 2]
                            h = min(float(point[2]) - CLEARANCE, self.top_limit(*rect))
                            score = h - 0.3 * abs(h - point[2]) + 0.02 * (depth + width)
                            if best is None or score > best[0]:
                                best = (score, rect, h, point.copy())
        _, rect, h, target = best
        if h <= self.ground + 0.015:
            raise ValueError(f"No elevated {kind} box fits the swept G1 bounds")
        self.support_box(rect, h, {"seat": "seat", "climb": "climb_box", "vault": "vault_box"}[kind])
        self.details.update(
            estimated_support_point=target.tolist(), support_point_to_box_top_gap_m=float(target[2] - h)
        )

    def jump(self, gap):
        lo, hi = self.bounds
        feet = self.feet
        mid = len(feet) // 2
        yl, yh = lo[1] - 0.25, hi[1] + 0.25
        if gap and np.ptp(self.pelvis[:, 0]) > 0.6:
            # Find a traversed interval where both feet have gained height relative
            # to the takeoff/landing baseline, then fit the two platform tops.
            sole = feet[..., 2].min(axis=(1, 2))
            baseline = np.linspace(sole[0], sole[-1], len(sole))
            airborne = np.flatnonzero(sole - baseline > 0.10)
            if len(airborne):
                first, last = int(airborne[0]), int(airborne[-1])
                edge_a = float(feet[first, ..., 0].min())
                edge_b = float(feet[last, ..., 0].max())
                if edge_b - edge_a > 0.15:
                    floor = self.ground
                    self.ground -= 0.6
                    self.support_box(
                        [lo[0] - 0.4, edge_a, yl, yh],
                        float(sole[: max(1, first)].min() - CLEARANCE),
                        "takeoff_platform",
                        self.ground - 0.15,
                    )
                    self.support_box(
                        [edge_b, hi[0] + 0.4, yl, yh],
                        float(sole[last:].min() - CLEARANCE),
                        "landing_platform",
                        self.ground - 0.15,
                    )
                    self.details.update(
                        gap_width_m=edge_b - edge_a,
                        estimated_flight_frames=[first, last],
                        original_motion_minimum_world_z_m=floor + CLEARANCE,
                    )
                    return
        # A jump up/down or in-place jump needs a support platform, not an invented gap.
        split = float(np.median(self.pelvis[:, 0]))
        if np.ptp(self.pelvis[:, 0]) > 0.25:
            self.support_box(
                [lo[0] - 0.4, split, yl, yh], float(feet[: max(1, mid), ..., 2].min() - CLEARANCE), "start_platform"
            )
            self.support_box(
                [split, hi[0] + 0.4, yl, yh], float(feet[mid:, ..., 2].min() - CLEARANCE), "finish_platform"
            )
        else:
            self.support_box([lo[0] - 0.3, hi[0] + 0.3, yl, yh], float(feet[..., 2].min() - CLEARANCE), "jump_platform")

    def ramp(self):
        feet = self.feet.mean(axis=2)
        feet[..., 2] = self.feet[..., 2].min(axis=2)
        intervals = support_intervals(self.feet, float(self.data["fps"]), 0.4)
        points = np.array([feet[i["start"] : i["stop"], i["foot"]].mean(axis=0) for i in intervals])
        if len(points) < 2:
            raise ValueError("Insufficient ramp support samples")
        a, b = np.polyfit(points[:, 0], points[:, 2], 1)
        angle = float(np.arctan(np.clip(a, -0.8, 0.8)))
        slope = float(np.tan(angle))
        lo, hi = self.bounds
        # Lower the plane until all swept body bounds clear it; this is the same
        # conservative geometry test as the existing passage construction.
        b = float(np.min(self.points[..., 2] - slope * self.points[..., 0]) - CLEARANCE)
        x = (lo[0] + hi[0]) / 2
        thickness = 0.1
        center = [x + np.sin(angle) * thickness / 2, (lo[1] + hi[1]) / 2, slope * x + b - np.cos(angle) * thickness / 2]
        self.add(
            center,
            [(hi[0] - lo[0] + 0.6) / np.cos(angle), hi[1] - lo[1] + 0.5, thickness],
            "ramp",
            Rotation.from_euler("y", -angle),
        )
        self.ground = min(self.ground, float(slope * (lo[0] - 0.3) + b) - 0.02)
        self.details.update(ramp_slope_degrees=float(np.rad2deg(angle)), ramp_top_plane_z_equals_ax_plus_b=[slope, b])


def fit(input_path, output_path, kind):
    with np.load(input_path, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    for key in (
        "terrain_boxes",
        "terrain_box_quat",
        "terrain_position",
        "terrain_dimension",
        "terrain_quat",
        "terrain_vertices",
        "terrain_faces",
    ):
        data.pop(key, None)
    fitter = TerrainFit(data, kind)
    if kind == "stairs":
        fitter.stairs()
    elif kind == "ladder":
        fitter.ladder()
    elif kind == "wall":
        fitter.wall()
    elif kind == "passage":
        fitter.passage()
    elif kind in ("seat", "climb", "vault"):
        fitter.contact_box(kind)
    elif kind in ("jump", "gap"):
        fitter.jump(kind == "gap")
    elif kind == "ramp":
        fitter.ramp()
    elif kind != "ground":
        raise ValueError(f"Unknown terrain kind: {kind}")
    fitter.floor()
    boxes = np.asarray(fitter.boxes)
    data.update(
        terrain_position=boxes[:, :3],
        terrain_dimension=boxes[:, 3:],
        terrain_quat=np.asarray(fitter.quats),
        ground_height=fitter.ground,
        terrain_id=np.asarray(f"mixamo_{input_path.parent.name}_{kind}"),
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output_path, **data)
    report = {
        "geometry_source": "static terrain fitted to G1 reference geometry; not supplied by Mixamo",
        "method": "existing USD swept-body bounds, foot-box support intervals and canonical terrain schema",
        "terrain_kind": kind,
        "terrain_roles": fitter.roles,
        "terrain_box_count": len(boxes),
        "world_frame": "unchanged motion frame; meters, Z-up; box quaternion order xyzw",
        "ground_top_world_z_m": fitter.ground,
        "motion_modified": False,
        "full_source_frame_count_preserved": len(data["joint_pos"]),
        "dynamic_tracking_validated": False,
        "contact_forces_measured": False,
        "ready_for_training": False,
        "source_motion_sha256": hashlib.sha256(input_path.read_bytes()).hexdigest(),
        "motion_sha256": hashlib.sha256(output_path.read_bytes()).hexdigest(),
        **fitter.details,
    }
    output_path.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Fitted {kind}: {input_path.parent.name}, {len(boxes)} boxes", flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--kind",
        required=True,
        choices=["stairs", "ladder", "passage", "seat", "climb", "vault", "jump", "gap", "ramp", "ground", "wall"],
    )
    args = p.parse_args()
    if args.input.resolve() == args.output.resolve():
        p.error("Preserve the raw reference; use a distinct output")
    fit(args.input, args.output, args.kind)


if __name__ == "__main__":
    main()
