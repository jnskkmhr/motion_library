# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Fit static stairs to an ascending +X motion using the exact G1 foot boxes.

This deliberately limited terrain fitter uses low foot speed to propose support
intervals. Kimodo's ground-contact labels do not identify elevated stair contacts.
It preserves the generated joint motion and only translates the world Z origin.
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path

import numpy as np


def support_intervals(corners: np.ndarray, fps: float, speed_threshold: float) -> list[dict]:
    """Find sustained low-speed foot intervals; these are geometric estimates, not forces."""
    speeds = np.linalg.norm(np.gradient(corners.mean(axis=2), 1 / fps, axis=0), axis=-1)
    intervals = []
    for foot in range(2):
        edges = np.diff(np.r_[False, speeds[:, foot] < speed_threshold, False].astype(int))
        for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            if stop - start >= max(3, int(np.ceil(0.12 * fps))):
                vertices = corners[start:stop, foot]
                intervals.append(
                    {
                        "foot": foot,
                        "start": int(start),
                        "stop": int(stop),
                        "x": float(vertices[..., 0].mean()),
                        "min_z": float(vertices[..., 2].min()),
                    }
                )
    return sorted(intervals, key=lambda interval: interval["x"])


def clear_front_edge(corners: np.ndarray, height: float) -> float:
    """Find the furthest +X point of any foot box below a proposed stair top.

    Clipping the twelve box edges against the height plane also catches collisions
    along a foot edge that would be missed by checking only the eight vertices.
    The result applies to axis-aligned stairs spanning the entire foot path in Y.
    """
    x_max = corners[..., 0][corners[..., 2] <= height].max(initial=-np.inf)
    bits = list(itertools.product((0, 1), repeat=3))
    for i, j in itertools.combinations(range(8), 2):
        if sum(a != b for a, b in zip(bits[i], bits[j])) != 1:
            continue
        a, b = corners[:, :, i], corners[:, :, j]
        crossing = (a[..., 2] - height) * (b[..., 2] - height) < 0
        if crossing.any():
            a, b = a[crossing], b[crossing]
            fraction = (height - a[:, 2]) / (b[:, 2] - a[:, 2])
            x_max = max(x_max, (a[:, 0] + fraction * (b[:, 0] - a[:, 0])).max())
    return float(x_max + 0.003)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=float, default=1.2, help="Stair width along world Y in meters.")
    parser.add_argument("--support_speed", type=float, default=0.18, help="Foot-center speed threshold in m/s.")
    args = parser.parse_args()
    if args.width <= 0 or args.support_speed <= 0 or args.input.resolve() == args.output.resolve():
        parser.error("Use positive width/speed and a distinct output path to preserve the source motion.")
    with np.load(args.input, allow_pickle=False) as source:
        data = {key: source[key] for key in source.files}
    corners = data["foot_box_corners_w"].copy()
    floor_offset = float(corners[..., 2].min())
    corners[..., 2] -= floor_offset
    data["body_pos_w"][..., 2] -= floor_offset
    if "hand_sphere_centers_w" in data:
        data["hand_sphere_centers_w"][..., 2] -= floor_offset
    data["foot_box_corners_w"] = corners
    intervals = support_intervals(corners, float(data["fps"]), args.support_speed)
    if len(intervals) < 2:
        raise ValueError(
            "No ascending stair sequence found; this fitter is not a general terrain reconstruction method."
        )
    y_center = float((corners[..., 1].max() + corners[..., 1].min()) / 2)
    if np.ptp(corners[..., 1]) > args.width:
        raise ValueError("Stair width must cover the entire foot path.")
    end_x = float(corners[..., 0].max() + 0.3)
    boxes, supports, gaps = [], np.zeros(corners.shape[:2], dtype=bool), []
    previous_height, previous_front = 0.0, -np.inf
    for interval in intervals:
        foot, start, stop = interval["foot"], interval["start"], interval["stop"]
        vertices = corners[start:stop, foot]
        # All boxes reach the ground. Stacked overlap forms monotonically ascending stairs.
        if interval["min_z"] < 0.06:
            height, front = 0.0, None
        else:
            proposed_height = interval["min_z"] - 0.002
            for height in np.arange(proposed_height, proposed_height - 0.041, -0.001):
                front = clear_front_edge(corners, height)
                # Keep the foot center at least 15 mm beyond the front edge during support.
                if front <= interval["x"] - 0.015:
                    break
            else:
                raise ValueError(f"Cannot fit collision-free support for interval {interval}.")
            if height <= previous_height + 0.04 or front <= previous_front:
                raise ValueError("Expected distinct stair heights and front edges increasing along +X.")
            boxes.append([(front + end_x) / 2, y_center, height / 2, end_x - front, args.width, height])
            previous_height, previous_front = height, front
        gap = vertices[..., 2].min(axis=-1) - height
        interval.update(height=float(height), front_x=front, max_sole_gap_m=float(gap.max()))
        gaps.extend(gap.tolist())
        supports[start:stop, foot] = True
    if not boxes:
        raise ValueError("The motion only contains ground-level support.")
    data.update(terrain_boxes=np.asarray(boxes, dtype=np.float32), estimated_foot_support=supports)
    joint_violation = float(
        np.maximum(data["joint_limits"][:, 0] - data["joint_pos"], data["joint_pos"] - data["joint_limits"][:, 1]).max()
    )
    report = {
        "robot_asset": str(data["robot_asset"].item()),
        "world_frame": "Z-up, meters; stairs ascend along +X",
        "motion_file": args.output.name,
        "terrain_boxes_columns": ["x", "y", "z", "size_x", "size_y", "size_z"],
        "terrain_boxes": data["terrain_boxes"].tolist(),
        "world_z_translation_m": -floor_offset,
        "frames": len(corners),
        "fps": float(data["fps"]),
        "support_speed_threshold_m_s": args.support_speed,
        "support_intervals": intervals,
        "max_estimated_support_sole_gap_m": max(gaps),
        "max_joint_limit_violation_rad": max(0.0, joint_violation),
        "foot_boxes_clear_fitted_stair_solids": True,
        "dynamic_tracking_validated": False,
        "limitations": [
            "Supplied/generative motion may float above stair tops within the reported support gap.",
            "No self-collision, non-foot terrain collision, torque, balance, or policy tracking validation.",
            "The default mimic task has a plane: training must instantiate these boxes and align reset transforms.",
        ],
    }
    if max(gaps) > 0.04 or joint_violation > 1e-4:
        raise ValueError(
            f"Candidate rejected: support gap={max(gaps):.4f} m, joint violation={joint_violation:.4f} rad"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **data)
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Saved {len(boxes)} stair boxes and mimic motion to {args.output}")
    print(f"Maximum estimated support sole gap: {max(gaps) * 1000:.1f} mm; dynamic tracking remains untested.")


if __name__ == "__main__":
    main()
