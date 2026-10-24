# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Fit a low passage to a crouching/crawling motion using conservative G1 mesh bounds.

The motion is unchanged. This fits collision-free overhead geometry to an existing
motion; it does not demonstrate that a policy responds to the new obstacle.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pxr import Usd, UsdGeom, UsdPhysics

from isaaclab_assets import UNITREE_G1_29DOF_BALL_HAND_CFG, UNITREE_G1_29DOF_BOX_FOOT_CFG


def body_geometry_corners(
    body_names: list[str], usd_path: str = UNITREE_G1_29DOF_BOX_FOOT_CFG.spawn.usd_path
) -> np.ndarray:
    """Bound visual and collision geometry in each rigid body's local frame."""
    stage = Usd.Stage.Open(usd_path)
    cache = UsdGeom.XformCache()
    points = {name: [] for name in body_names}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            vertices = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        elif prim.IsA(UsdGeom.Cube):
            half = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
            vertices = np.asarray(list(itertools.product((-half, half), repeat=3)))
        elif prim.IsA(UsdGeom.Sphere):
            radius = UsdGeom.Sphere(prim).GetRadiusAttr().Get()
            vertices = np.asarray(list(itertools.product((-radius, radius), repeat=3)))
        else:
            continue
        parent = prim.GetParent()
        while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):
            parent = parent.GetParent()
        if not parent or parent.GetName() not in points:
            raise ValueError(f"Cannot associate robot geometry with a rigid body: {prim.GetPath()}")
        # USD uses row-vector transforms; transpose for NumPy column-vector application.
        local = np.asarray(cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(parent).GetInverse()).T
        points[parent.GetName()].append(vertices @ local[:3, :3].T + local[:3, 3])
    corners = []
    for name in body_names:
        if not points[name]:
            raise ValueError(f"No geometry found for body {name}")
        vertices = np.concatenate(points[name])
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        corners.append(list(itertools.product(*zip(lo, hi))))
    return np.asarray(corners)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--clearance", type=float, default=0.04, help="Roof clearance above robot mesh bounds, meters.")
    args = parser.parse_args()
    if args.clearance <= 0 or args.input.resolve() == args.output.resolve():
        parser.error("Use positive clearance and a distinct output path.")
    with np.load(args.input, allow_pickle=False) as source:
        data = {key: source[key] for key in source.files}
    assets = {
        "UNITREE_G1_29DOF_BOX_FOOT_CFG": UNITREE_G1_29DOF_BOX_FOOT_CFG,
        "UNITREE_G1_29DOF_BALL_HAND_CFG": UNITREE_G1_29DOF_BALL_HAND_CFG,
    }
    asset = assets[str(data["robot_asset"].item())]
    local = body_geometry_corners(data["body_names"].tolist(), asset.spawn.usd_path)
    rotations = (
        Rotation.from_quat(data["body_quat_w"].reshape(-1, 4)).as_matrix().reshape(*data["body_quat_w"].shape[:2], 3, 3)
    )
    corners = np.einsum("tbij,bcj->tbci", rotations, local) + data["body_pos_w"][:, :, None, :]
    low, high = corners.min(axis=2), corners.max(axis=2)
    pelvis = data["body_pos_w"][:, data["body_names"].tolist().index("pelvis")]
    travel = pelvis[-1, 0] - pelvis[0, 0]
    if travel < 1.0 or np.ptp(pelvis[:, 1]) > 0.5:
        raise ValueError("This passage fitter expects at least 1 m of travel along +X with little lateral drift.")
    # Place the passage in the middle of the path, after the initial posture transition.
    x_start, x_end = pelvis[0, 0] + travel * np.array([0.45, 0.65])
    intersects_x = (high[..., 0] >= x_start) & (low[..., 0] <= x_end)
    roof_bottom = float(high[..., 2][intersects_x].max() + args.clearance)
    standing_height = float(high[0, :, 2].max())
    if roof_bottom >= standing_height - 0.1:
        raise ValueError("The fitted passage is not meaningfully lower than the starting standing posture.")
    y_lo, y_hi = low[..., 1].min() - 0.12, high[..., 1].max() + 0.12
    y_mid, width = (y_lo + y_hi) / 2, y_hi - y_lo
    thickness = 0.06
    boxes = [[(x_start + x_end) / 2, y_mid, roof_bottom + thickness / 2, x_end - x_start, width, thickness]]
    for x in (x_start, x_end):
        for y in (y_lo, y_hi):
            boxes.append([x, y, roof_bottom / 2, thickness, thickness, roof_bottom])
    data["terrain_boxes"] = np.asarray(boxes, dtype=np.float32)
    data["terrain_box_quat"] = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (len(boxes), 1))
    # Independently check separating world axes against every transformed body bound.
    min_separation = np.inf
    for box in data["terrain_boxes"]:
        box_lo, box_hi = box[:3] - box[3:] / 2, box[:3] + box[3:] / 2
        separation = np.maximum(low - box_hi, box_lo - high).max(axis=-1)
        min_separation = min(min_separation, float(separation.min()))
    if min_separation < -1e-5:
        raise ValueError("Fitted passage intersects a robot body bound.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **data)
    report = {
        "robot_asset": str(data["robot_asset"].item()),
        "geometry_source": "new passage fitted to motion and target robot mesh bounds",
        "motion_modified": False,
        "roof_underside_height_m": roof_bottom,
        "standing_mesh_height_m": standing_height,
        "roof_clearance_m": args.clearance,
        "minimum_body_bound_separation_from_passage_m": min_separation,
        "passage_length_m": float(x_end - x_start),
        "passage_width_m": float(width),
        "full_source_frame_count_preserved": len(pelvis),
        "dynamic_tracking_validated": False,
        "ground_contact_validated": False,
        "ending": "Original clip ending preserved; no stand-up or stopping transition added.",
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Fitted passage: roof underside {roof_bottom:.3f} m, standing mesh height {standing_height:.3f} m")


if __name__ == "__main__":
    main()
