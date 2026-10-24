# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate terrain-first G1 batches and refine estimated support contacts.

Run prepare/generate/refine/validate in the Kimodo environment with MuJoCo.
Terrain is fixed before generation. Support intervals except jump takeoff/landing
are estimated from generated geometry and speed, not measured contact forces.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import binary_closing, label
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from terrain_platform import PlatformKinematics


class ContactKinematics(PlatformKinematics):
    """G1 whole-body kinematics and six possible support contacts in a fixed scene."""

    def __init__(self, mjcf: Path, scene: dict):
        super().__init__(mjcf, scene)
        self.scene = scene
        self.variables = [0, 1, 2, *range(7, 36)]
        self.lower = np.r_[[-3, -2, 0.15], self.limits[:, 0]]
        self.upper = np.r_[[5, 2, 2.5], self.limits[:, 1]]
        self.contact_geoms = self.foot_geoms + [self.model.geom(f"{s}_ball_pad").id for s in ("left", "right")]
        self.contact_geoms += [self.model.geom(f"{s}_knee_link_collision").id for s in ("left", "right")]
        self.contact_bodies = self.model.geom_bodyid[self.contact_geoms]
        self.support_surfaces = [self.surfaces[i] for i in scene["support_surface_indices"]]

    def forward(self, qpos: np.ndarray):
        """Update body and collision-geometry transforms without solving dynamics."""
        self.data.qpos[:] = qpos
        mujoco.mj_kinematics(self.model, self.data)

    def solve_contacts(self, reference, local_points, targets, active, previous=None, foot_rotations=None):
        """Use bounded IK to enforce support anchors and reduce terrain penetration."""
        q = reference.copy()
        q[7:] = np.clip(q[7:], self.limits[:, 0], self.limits[:, 1])
        original = q[self.variables].copy()
        previous = np.zeros(32) if previous is None else previous

        def residual(values):
            q[self.variables] = values
            self.forward(q)
            matrices = self.data.xmat[self.contact_bodies].reshape(6, 3, 3)
            points = self.data.xpos[self.contact_bodies] + np.einsum("lij,lj->li", matrices, local_points)
            distances = [
                mujoco.mj_geomDistance(self.model, self.data, g, s, 0.08, None)
                for g in self.collision_geoms
                for s in self.surfaces
            ]
            errors = [
                250 * ((points - targets) * active[:, None]).ravel(),
                1200 * np.minimum(np.asarray(distances) - 0.001, 0),
                np.r_[15, 15, 15, np.full(29, 0.8)] * (values - original),
                1.5 * (values - original - previous),
            ]
            if foot_rotations is not None:
                rotation_error = Rotation.from_matrix(matrices[:2] @ foot_rotations.transpose(0, 2, 1)).as_rotvec()
                errors.append((20 * rotation_error * active[:2, None]).ravel())
            if self.scene["name"] == "bear_walk":
                knee_gaps = [
                    mujoco.mj_geomDistance(self.model, self.data, g, self.surfaces[0], 0.1, None)
                    for g in self.contact_geoms[4:]
                ]
                errors.append(200 * np.minimum(np.asarray(knee_gaps) - 0.035, 0))
            return np.concatenate(errors)

        result = least_squares(
            residual,
            np.clip(original + previous, self.lower, self.upper),
            bounds=(self.lower, self.upper),
            max_nfev=65,
            ftol=1e-6,
        )
        q[self.variables] = result.x
        return q.copy(), result.x - original

    def nearest_support(self, contact):
        """Return signed gap, closest robot point, closest terrain point and surface ID."""
        geom = self.contact_geoms[contact]
        # A foot inside the ramp footprint must use its top, not the ground hidden
        # beneath it, even when the generated foot currently penetrates the ramp.
        center = self.data.geom_xpos[geom]
        below = []
        for surface in self.support_surfaces:
            if surface == self.surfaces[0]:
                below.append((0.0, surface))
                continue
            matrix = self.data.geom_xmat[surface].reshape(3, 3)
            normal = matrix[:, 2]
            top = self.data.geom_xpos[surface] + normal * self.model.geom_size[surface, 2]
            z = top[2] - np.dot(normal[:2], center[:2] - top[:2]) / normal[2]
            projected = np.r_[center[:2], z]
            local = matrix.T @ (projected - self.data.geom_xpos[surface])
            if np.all(np.abs(local[:2]) <= self.model.geom_size[surface, :2] + 1e-6):
                below.append((float(z), surface))
        surfaces = [max(below)[1]] if below else self.support_surfaces
        best = None
        for surface in surfaces:
            segment = np.zeros(6)
            distance = mujoco.mj_geomDistance(self.model, self.data, geom, surface, 2, segment)
            if best is None or abs(distance) < abs(best[0]):
                best = (distance, segment[:3].copy(), segment[3:].copy(), surface)
        return best


def make_scene(item, mjcf):
    """Specify visible boxes and support surfaces without looking at a generated motion."""
    scene = dict(item)
    name = item["name"]
    if name in ["crouch", "crawl", "bear_walk"]:
        h = item["roof_height"]
        start, end = 0.45, 1.05
        boxes = [[(start + end) / 2, 0, h + 0.04, end - start, 1.2, 0.08]]
        boxes += [[x, y, h / 2, 0.06, 0.06, h] for x in [start, end] for y in [-0.6, 0.6]]
        quats = [[0, 0, 0, 1]] * len(boxes)
        support = [0]
    elif name == "ramp":
        theta = np.deg2rad(item["slope_degrees"])
        boxes = [[0.95, 0, np.tan(theta) * (0.95 - 0.2) - 0.05 / np.cos(theta), 1.6, 1.2, 0.1]]
        quats = [Rotation.from_euler("y", -theta).as_quat().tolist()]
        support = [0, 1]
    elif name == "jump_gap":
        h = item["platform_height"]
        gap = item["gap_width"]
        scene.update(takeoff_time=item.get("takeoff_time", 1.5), landing_time=item.get("landing_time", 2.3))
        boxes = [[-0.7, 0, h / 2, 2, 1.4, h], [0.3 + gap + 1, 0, h / 2, 2, 1.4, h]]
        quats = [[0, 0, 0, 1]] * 2
        support = [1, 2]
    else:
        raise ValueError(f"Unknown scenario: {name}")
    scene.update(
        terrain_boxes=boxes,
        terrain_box_quat=quats,
        support_surface_indices=support,
        coordinate_system="Z-up, X-forward, meters",
        geometry_source="specified_before_generation",
        mjcf=str(mjcf.resolve()),
        mjcf_sha256=hashlib.sha256(mjcf.read_bytes()).hexdigest(),
    )
    return scene


def posture(robot, name):
    """Initial posture guess for IK, expressed in named robot joints."""
    q = robot.model.qpos0.copy()
    q[:3] = [0, 0, 0.75]
    q[3:7] = [1, 0, 0, 0]
    values = {"hip_pitch": -0.312, "knee": 0.669, "ankle_pitch": -0.357, "elbow": 0.6}
    if name == "crouch":
        q[2] = 0.50
        q[3:7] = Rotation.from_euler("y", 0.35).as_quat()[[3, 0, 1, 2]]
        values.update(hip_pitch=-1.6, knee=2.0, ankle_pitch=-0.75, shoulder_pitch=-0.5)
    elif name == "crawl":
        q[2] = 0.35
        q[3:7] = Rotation.from_euler("y", 1.07).as_quat()[[3, 0, 1, 2]]
        values.update(hip_pitch=-1.1, knee=1.95, ankle_pitch=0.43, waist_pitch=0.5, shoulder_pitch=-1.65, elbow=1.05)
    elif name == "bear_walk":
        q[2] = 0.49
        q[3:7] = Rotation.from_euler("y", 1.4).as_quat()[[3, 0, 1, 2]]
        values.update(hip_pitch=-2.1, knee=1.5, ankle_pitch=-0.75, waist_pitch=0.2, shoulder_pitch=-1.6, elbow=0.25)
    for i, n in enumerate(robot.names):
        for pattern, angle in values.items():
            if pattern + "_joint" in n:
                q[7 + i] = angle
        if "shoulder_roll" in n:
            q[7 + i] = 0.25 if n.startswith("left") else -0.25
    return q


def resting_pose(robot, guess, floor_height=0, foot_rotation=None):
    """Construct a support-consistent constraint pose from a posture guess."""
    robot.forward(guess)
    local = np.zeros((6, 3))
    targets = np.zeros((6, 3))
    active = np.zeros(6)
    contacts = (
        [2, 3, 4, 5]
        if robot.scene["name"] == "crawl"
        else ([0, 1, 2, 3] if robot.scene["name"] == "bear_walk" else [0, 1])
    )
    for j in contacts:
        body = robot.contact_bodies[j]
        geom = robot.contact_geoms[j]
        if j < 2:
            local[j] = robot.model.geom_pos[geom] - [0, 0, robot.model.geom_size[geom, 2]]
            targets[j] = robot.data.xpos[body] + robot.data.xmat[body].reshape(3, 3) @ local[j]
            targets[j, 2] = floor_height
        elif j < 4:
            local[j] = robot.model.geom_pos[geom]
            targets[j] = robot.data.geom_xpos[geom]
            targets[j, 2] = floor_height + robot.model.geom_size[geom, 0]
        else:
            _, point, target, _ = robot.nearest_support(j)
            local[j] = robot.data.xmat[body].reshape(3, 3).T @ (point - robot.data.xpos[body])
            targets[j] = target
        active[j] = 1
    rotations = np.tile(np.eye(3) if foot_rotation is None else foot_rotation, (2, 1, 1))
    if foot_rotation is not None:
        for j in range(2):
            targets[j, 2] = np.tan(np.deg2rad(robot.scene["slope_degrees"])) * (targets[j, 0] - 0.2)
    q, _ = robot.solve_contacts(
        guess, local, targets, active, foot_rotations=rotations if robot.scene["name"] != "crawl" else None
    )
    return q


def prepare_case(item, mjcf, output):
    """Write one fixed scene and sparse full-body constraints for all seeds."""
    import xml.etree.ElementTree as ET

    from kimodo.exports.mujoco import MujocoQposConverter
    from kimodo.skeleton import G1Skeleton34

    scene = make_scene(item, mjcf)
    robot = ContactKinematics(mjcf, scene)
    n = round(item["duration"] * 30)
    name = item["name"]
    if name == "jump_gap":
        takeoff, landing = scene["takeoff_time"], scene["landing_time"]
        apex = (takeoff + landing) / 2
        # A ballistic-height apex guides the reference; this does not enforce dynamics.
        rise = 9.81 * (landing - takeoff) ** 2 / 8
        times = [0, takeoff - 0.7, takeoff - 0.2, takeoff, apex, landing, landing + 0.2, landing + 1, (n - 1) / 30]
        frames = [round(t * 30) for t in times]
        if len(set(frames)) != len(frames) or sorted(frames) != frames:
            raise ValueError("Jump timing must leave room for preparation and landing recovery")
        end = item["travel"]
        xs = [0, 0.02, 0.06, 0.1, (0.1 + end) / 2, end, end, end, end]
        heights = [0.75, 0.65, 0.51, 0.75, 0.75 + rise, 0.75, 0.54, 0.73, 0.75]
        knees = [0.67, 1.1, 1.9, 0.55, 1.2, 0.8, 1.75, 0.75, 0.67]
        shoulder_pitch = [0.05, 0.5, 0.9, -1.5, -1.9, -0.85, -0.6, -0.1, 0.05]
        root_pitch = [0, 0.15, 0.4, 0.12, 0.05, 0.22, 0.3, 0.05, 0]
        poses = []
        for index, (x, z, knee, arm, pitch) in enumerate(
            zip(xs, heights, knees, shoulder_pitch, root_pitch, strict=True)
        ):
            q = posture(robot, "standing")
            q[0] = x
            q[2] = z + item["platform_height"]
            q[3:7] = Rotation.from_euler("y", pitch).as_quat()[[3, 0, 1, 2]]
            for i, joint in enumerate(robot.names):
                if "hip_pitch" in joint:
                    q[7 + i] = -knee / 2 - pitch
                elif "knee" in joint:
                    q[7 + i] = knee
                elif "ankle_pitch" in joint:
                    q[7 + i] = -knee / 2
                elif "shoulder_pitch" in joint:
                    q[7 + i] = arm
                elif "elbow" in joint:
                    q[7 + i] = 0.45
                elif "shoulder_roll" in joint:
                    q[7 + i] = 0.12 if joint.startswith("left") else -0.12
            if index != 4:
                q = resting_pose(robot, q, item["platform_height"])
            poses.append(q)
    else:
        frames = [0, 45, 90, 135, n - 1] if name == "crouch" else [0, 60, 120, n - 1]
        poses = []
        for f in frames:
            x = item["travel"] * f / (n - 1)
            style = "standing" if name == "crouch" and f in [0, n - 1] else name
            q = posture(robot, style)
            q[0] = x
            if name == "ramp" and x > 0.2:
                q[2] += np.tan(np.deg2rad(item["slope_degrees"])) * (x - 0.2)
                q = resting_pose(
                    robot, q, foot_rotation=Rotation.from_euler("y", -np.deg2rad(item["slope_degrees"])).as_matrix()
                )
            else:
                q = resting_pose(robot, q)
            poses.append(q)
    converter = MujocoQposConverter(G1Skeleton34())
    cnames = [j.attrib["name"] for j in ET.parse(converter.xml_path).find("worldbody").iter("joint")]
    poses = np.asarray(poses)
    poses[:, 7:] = poses[:, [7 + robot.names.index(n) for n in cnames]]
    motion = converter.qpos_to_motion_dict(np.repeat(poses, 30, axis=0).astype(np.float32), 30)
    matrices = np.asarray(motion["local_rot_mats"])[::30]
    roots = np.asarray(motion["root_positions"])[::30]
    constraints = [
        {
            "type": "fullbody",
            "frame_indices": frames,
            "local_joints_rot": Rotation.from_matrix(matrices.reshape(-1, 3, 3))
            .as_rotvec()
            .reshape(-1, 34, 3)
            .tolist(),
            "root_positions": roots.tolist(),
            "smooth_root_2d": roots[:, [0, 2]].tolist(),
        }
    ]
    output.mkdir(parents=True, exist_ok=True)
    for filename, value in [("terrain.json", scene), ("constraints.json", constraints)]:
        (output / filename).write_text(json.dumps(value, indent=2) + "\n")


def contact_plan(robot, poses, fps):
    """Estimate stationary support intervals and project their anchors onto fixed terrain."""
    count = len(poses)
    positions = np.zeros((count, 6, 3))
    gaps = np.zeros((count, 6))
    local = np.zeros((count, 6, 3))
    targets = np.zeros_like(local)
    surface_ids = np.zeros((count, 6), int)
    rotations = np.tile(np.eye(3), (count, 2, 1, 1))
    name = robot.scene["name"]
    eligible = [0, 1, 2, 3] if name == "bear_walk" else ([2, 3, 4, 5] if name == "crawl" else [0, 1])
    for i, q in enumerate(poses):
        robot.forward(q)
        for j in eligible:
            body = robot.contact_bodies[j]
            geom = robot.contact_geoms[j]
            gap, point, target, surface = robot.nearest_support(j)
            gaps[i, j] = gap
            surface_ids[i, j] = surface
            if j < 2:
                local[i, j] = robot.model.geom_pos[geom] - [0, 0, robot.model.geom_size[geom, 2]]
                point = robot.data.xpos[body] + robot.data.xmat[body].reshape(3, 3) @ local[i, j]
                normal = robot.data.geom_xmat[surface].reshape(3, 3)[:, 2]
                point_surface = robot.data.geom_xpos[surface] + normal * robot.model.geom_size[surface, 2]
                if surface == robot.surfaces[0]:
                    point_surface = np.zeros(3)
                target = point - normal * np.dot(point - point_surface, normal)
                yaw = Rotation.from_matrix(robot.data.xmat[body].reshape(3, 3)).as_euler("xyz")[2]
                rotations[i, j] = (
                    robot.data.geom_xmat[surface].reshape(3, 3) @ Rotation.from_euler("z", yaw).as_matrix()
                )
            elif j < 4:
                local[i, j] = robot.model.geom_pos[geom]
                point = robot.data.geom_xpos[geom].copy()
                target = point.copy()
                target[2] = robot.model.geom_size[geom, 0]
            else:
                local[i, j] = robot.data.xmat[body].reshape(3, 3).T @ (point - robot.data.xpos[body])
            positions[i, j] = point
            targets[i, j] = target
    speeds = np.linalg.norm(np.gradient(positions, 1 / fps, axis=0), axis=-1)
    active = np.zeros((count, 6), bool)
    for j in eligible:
        threshold = 0.18 if j >= 2 else 0.35
        mask = (speeds[:, j] < threshold) & (gaps[:, j] < 0.12) & (gaps[:, j] > -0.15)
        mask = binary_closing(mask, structure=np.ones(3))
        mask[:2] = (speeds[:2, j] < threshold) & (gaps[:2, j] < 0.12)
        mask[-2:] = (speeds[-2:, j] < threshold) & (gaps[-2:, j] < 0.12)
        if name == "jump_gap":
            times = np.arange(count) / fps
            mask = (times <= robot.scene.get("takeoff_time", 1.35)) | (times >= robot.scene.get("landing_time", 2.08))
        labels, total = label(mask)
        for region in range(1, total + 1):
            ids = np.flatnonzero(labels == region)
            if len(ids) < 4:
                continue
            # Split runs at surface changes rather than anchoring across an obstacle edge.
            for surface in np.unique(surface_ids[ids, j]):
                subset = ids[surface_ids[ids, j] == surface]
                if len(subset) < 4:
                    continue
                middle = subset[len(subset) // 2]
                local[subset, j] = local[middle, j]
                targets[subset, j] = np.median(targets[subset, j], axis=0)
                active[subset, j] = True
                if j < 2:
                    rotations[subset, j] = rotations[middle, j]
    return local, targets, active, rotations


def refine_case(output, mjcf):
    """Refine at final 50 Hz and retain original generated data with the same terrain."""
    scene = json.loads((output / "terrain.json").read_text())
    robot = ContactKinematics(mjcf, scene)
    if hashlib.sha256(mjcf.read_bytes()).hexdigest() != scene["mjcf_sha256"]:
        raise ValueError("MJCF changed")
    with np.load(output / "g1_qpos.npz", allow_pickle=False) as src:
        data = {k: src[k] for k in src.files}
    names = data["joint_names"].tolist()
    q = data["qpos"]
    t0 = np.arange(len(q)) / float(data["fps"])
    times = np.arange(int(t0[-1] * 50) + 1) / 50
    data.update(
        terrain_boxes=np.asarray(scene["terrain_boxes"]), terrain_box_quat=np.asarray(scene["terrain_box_quat"])
    )
    np.savez_compressed(output / "g1_terrain_qpos.npz", **data)
    quat = Slerp(t0, Rotation.from_quat(q[:, 3:7]))(times).as_quat()
    poses = np.c_[
        CubicSpline(t0, q[:, :3])(times),
        quat[:, [3, 0, 1, 2]],
        CubicSpline(t0, q[:, [7 + names.index(n) for n in robot.names]])(times),
    ]
    local, targets, active, rotations = contact_plan(robot, poses, 50)
    result = []
    previous = None
    for i, pose in enumerate(poses):
        corrected, previous = robot.solve_contacts(pose, local[i], targets[i], active[i], previous, rotations[i])
        result.append(corrected)
    result = np.asarray(result)
    data["qpos"] = np.c_[result[:, :3], result[:, [4, 5, 6, 3]], result[:, [7 + robot.names.index(n) for n in names]]]
    data["fps"] = 50
    # Retain original model labels only in the generated file. They do not describe the refined supports.
    data.pop("foot_contacts", None)
    np.savez_compressed(output / "g1_refined_qpos.npz", **data)
    np.savez_compressed(
        output / "contact_plan.npz",
        times=times,
        local_points=local,
        targets=targets,
        active=active,
        contact_names=["left_foot", "right_foot", "left_hand_ball", "right_hand_ball", "left_knee", "right_knee"],
    )
    report = {
        "refinement_version": 2,
        "contact_speed_threshold_m_s": {"feet": 0.35, "hands_and_knees": 0.18},
        "collision_clearance_target_m": 0.001,
        "terrain_modified": False,
        "contact_source": "geometry/speed heuristic; prescribed jump support windows",
        "refinement_fps": 50,
        "dynamic_tracking_validated": False,
        "max_root_correction_m": float(np.linalg.norm(result[:, :3] - poses[:, :3], axis=-1).max()),
        "max_joint_correction_rad": float(np.abs(result[:, 7:] - poses[:, 7:]).max()),
    }
    (output / "refinement.json").write_text(json.dumps(report, indent=2) + "\n")


def validate_case(output, mjcf):
    """Check sampled robot-terrain distances, contact anchors and action-specific requirements."""
    scene = json.loads((output / "terrain.json").read_text())
    robot = ContactKinematics(mjcf, scene)
    plan = np.load(output / "contact_plan.npz")
    active = plan["active"]
    local = plan["local_points"]
    targets = plan["targets"]
    report = {
        "dynamic_tracking_validated": False,
        "self_collision_validated": False,
        "terrain_modified": False,
        "collision_method": "MJCF primitives and convex mesh hulls at exported frames",
    }
    for kind, file in [("generated", "motion_generated.npz"), ("refined", "motion.npz")]:
        with np.load(output / file, allow_pickle=False) as d:
            names = d["joint_names"].tolist()
            bodies = d["body_names"].tolist()
            root = bodies.index("pelvis")
            for key in ["joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"]:
                if not np.isfinite(d[key]).all():
                    raise ValueError(f"Nonfinite {key}")
            if not np.allclose(d["terrain_boxes"], scene["terrain_boxes"]):
                raise ValueError("Terrain changed")
            if not np.allclose(d["terrain_box_quat"], scene["terrain_box_quat"]):
                raise ValueError("Terrain orientation changed")
            errors = []
            minimum = 0.1
            worst = None
            hand_gaps = []
            knee_gaps = []
            fk_error = 0
            for i in range(len(d["joint_pos"])):
                q = np.r_[
                    d["body_pos_w"][i, root],
                    d["body_quat_w"][i, root][[3, 0, 1, 2]],
                    d["joint_pos"][i, [names.index(n) for n in robot.names]],
                ]
                robot.forward(q)
                matrices = robot.data.xmat[robot.contact_bodies].reshape(6, 3, 3)
                points = robot.data.xpos[robot.contact_bodies] + np.einsum("lij,lj->li", matrices, local[i])
                errors.extend(np.linalg.norm(points - targets[i], axis=-1)[active[i]].tolist())
                for side, body in enumerate(robot.feet):
                    fk_error = max(
                        fk_error,
                        float(
                            np.max(
                                np.abs(
                                    robot.data.xpos[body]
                                    - d["body_pos_w"][i, bodies.index(robot.model.body(body).name)]
                                )
                            )
                        ),
                    )
                for geom in robot.collision_geoms:
                    for surface in robot.surfaces:
                        dist = mujoco.mj_geomDistance(robot.model, robot.data, geom, surface, 0.1, None)
                        if dist < minimum:
                            minimum = dist
                            worst = [i, robot.model.geom(geom).name, robot.model.geom(surface).name]
                hand_gaps.append([robot.nearest_support(j)[0] for j in [2, 3]])
                knee_gaps.append([robot.nearest_support(j)[0] for j in [4, 5]])
            if fk_error > 1e-4:
                raise ValueError(f"MJCF/USDA FK mismatch {fk_error}")
            limits = d["joint_limits"]
            violation = max(0, float(np.maximum(limits[:, 0] - d["joint_pos"], d["joint_pos"] - limits[:, 1]).max()))
            metrics = {
                "frames": len(d["joint_pos"]),
                "fps": float(d["fps"]),
                "max_robot_terrain_penetration_m": max(0, -minimum),
                "worst_pair": worst,
                "max_support_anchor_error_m": max(errors, default=None),
                "mean_support_anchor_error_m": float(np.mean(errors)) if errors else None,
                "max_joint_limit_violation_rad": violation,
                "max_mjcf_usda_ankle_fk_difference_m": fk_error,
                "max_joint_speed_rad_s": float(np.abs(d["joint_vel"]).max()),
                "forward_travel_m": float(d["body_pos_w"][-1, root, 0] - d["body_pos_w"][0, root, 0]),
                "hand_near_support_fraction": np.mean(np.abs(hand_gaps) < 0.015, axis=0).tolist(),
                "knee_near_support_fraction": np.mean(np.abs(knee_gaps) < 0.015, axis=0).tolist(),
                "minimum_knee_support_gap_m": float(np.min(knee_gaps)),
            }
            reasons = []
            if minimum < -0.002:
                reasons.append("robot-terrain penetration exceeds 2 mm")
            if violation > 1e-4:
                reasons.append("joint-limit violation")
            if not errors or max(errors) > 0.015:
                reasons.append("support anchor error exceeds 15 mm or no support detected")
            if metrics["forward_travel_m"] < 0.7 * scene["travel"]:
                reasons.append("insufficient forward travel")
            if scene["name"] in ["crawl", "bear_walk"] and min(metrics["hand_near_support_fraction"]) < 0.15:
                reasons.append("insufficient hand support")
            if scene["name"] == "crawl" and min(metrics["knee_near_support_fraction"]) < 0.15:
                reasons.append("insufficient knee support")
            if scene["name"] == "bear_walk" and metrics["minimum_knee_support_gap_m"] < 0.015:
                reasons.append("knees not kept off ground")
            if scene["name"] == "jump_gap":
                # Both feet must finish over the landing platform, and rise above takeoff height in flight.
                feet = d["foot_box_corners_w"]
                edge = 0.3 + scene["gap_width"]
                metrics["landed_on_far_platform"] = bool(
                    feet[-1, ..., 0].min() > edge
                    and np.abs(feet[-1, ..., 2].min(axis=-1) - scene["platform_height"]).max() < 0.02
                )
                metrics["both_feet_airborne_frames"] = int(
                    np.sum(np.min(feet[..., 2], axis=(1, 2)) > scene["platform_height"] + 0.08)
                )
                if not metrics["landed_on_far_platform"] or metrics["both_feet_airborne_frames"] < 5:
                    reasons.append("gap jump/landing not established")
            metrics["passes_geometric_checks"] = not reasons
            metrics["rejection_reasons"] = reasons
            report[kind] = metrics
    report["estimated_contact_fraction"] = active.mean(axis=0).tolist()
    (output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(output.name, scene["name"], report["refined"]["rejection_reasons"], flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["prepare", "generate", "refine", "validate"], required=True)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("terrain_batch.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--only", nargs="+")
    parser.add_argument("--seeds", nargs="+", type=int, help="Select a subset of manifest seeds.")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if args.seeds:
        if set(args.seeds) - set(manifest["seeds"]):
            parser.error("Unknown seed")
        manifest["seeds"] = args.seeds
    items = [m for m in manifest["motions"] if not args.only or m["name"] in args.only]
    if args.only and set(args.only) - {m["name"] for m in items}:
        parser.error("Unknown scenario")
    if args.stage == "prepare":
        if args.output.exists() and any(args.output.iterdir()):
            parser.error("Use a new output directory")
        for item in items:
            prepare_case(item, args.mjcf, args.output / item["name"])
        manifest["motions"] = items
        (args.output / "requests.json").write_text(json.dumps(manifest, indent=2) + "\n")
    elif args.stage == "generate":
        from generate import save_motion
        from kimodo import load_model
        from kimodo.constraints import load_constraints_lst
        from kimodo.tools import seed_everything

        model = load_model("Kimodo-G1-RP-v1", device="cuda")
        for item in items:
            source = args.output / item["name"]
            constraints = load_constraints_lst(str(source / "constraints.json"), model.skeleton)
            for seed in manifest["seeds"]:
                dest = source / f"seed_{seed}"
                if dest.exists():
                    raise FileExistsError(dest)
                seed_everything(seed)
                batch = model(
                    [item["prompt"]],
                    [round(item["duration"] * model.fps)],
                    num_samples=1,
                    num_denoising_steps=100,
                    constraint_lst=constraints,
                    post_processing=False,
                    return_numpy=True,
                )
                motion = {k: v[0] for k, v in batch.items() if isinstance(v, np.ndarray)}
                save_motion(
                    motion,
                    model.skeleton,
                    model.fps,
                    dest,
                    {
                        "model": "Kimodo-G1-RP-v1",
                        "source_type": "generated",
                        "prompt": item["prompt"],
                        "seed": seed,
                        "duration": item["duration"],
                        "terrain_source": "specified_before_generation",
                        "constraints_sha256": hashlib.sha256((source / "constraints.json").read_bytes()).hexdigest(),
                        "dynamic_tracking_validated": False,
                    },
                )
                for file in ["terrain.json", "constraints.json"]:
                    (dest / file).write_bytes((source / file).read_bytes())
    else:
        for item in items:
            for seed in manifest["seeds"]:
                dest = args.output / item["name"] / f"seed_{seed}"
                (refine_case if args.stage == "refine" else validate_case)(dest, args.mjcf)


if __name__ == "__main__":
    main()
