"""Report joint-limit excess and foot/hand geometry without claiming dynamic feasibility."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from data_generator.deep_wholebody_parkour.import_deep_parkour import load_robot


def screen(path, mjcf):
    model = load_robot(mjcf)
    with np.load(path, allow_pickle=False) as d:
        names = d["joint_names"].tolist()
        ids = [model.joint(n).id for n in names]
        excess = np.maximum(model.jnt_range[ids, 0] - d["joint_pos"], d["joint_pos"] - model.jnt_range[ids, 1])
        maximum = np.maximum(excess.max(axis=0), 0)
        report = {
            "frames": len(d["joint_pos"]),
            "fps": float(d["fps"]),
            "joint_limit_excess_rad": {n: float(v) for n, v in zip(names, maximum) if v > 1e-5},
            "max_joint_speed_rad_s": float(abs(d["joint_vel"]).max()),
            "min_foot_corner_world_z_m": float(d["foot_box_corners_w"][..., 2].min()),
            "dynamic_tracking_validated": False,
            "ready_for_training": False,
            "note": "Sampled joint/geometry screening only; no full-body collision or dynamics validation.",
        }
        # Vertical depth below the highest box top with the same reference XY.
        if "terrain_position" in d and "foot_box_corners_w" in d:
            points = d["foot_box_corners_w"].reshape(-1, 3)
            depths = []
            quats = d["terrain_quat"] if "terrain_quat" in d else np.tile([0, 0, 0, 1], (len(d["terrain_position"]), 1))
            for pos, dim, quat in zip(d["terrain_position"], d["terrain_dimension"], quats):
                local = Rotation.from_quat(quat).inv().apply(points - pos)
                inside = np.all(abs(local[:, :2]) <= dim[:2] / 2, axis=1) & (local[:, 2] >= -dim[2] / 2)
                if inside.any():
                    depths.append(float(np.maximum(dim[2] / 2 - local[inside, 2], 0).max()))
            report["max_foot_corner_depth_below_box_top_m"] = max(depths, default=0)
            report["depth_definition"] = (
                "Local box-Z depth at corners whose XY lies inside the box footprint; not minimum collision penetration."
            )
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--motion", type=Path, required=True)
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.write_text(json.dumps(screen(a.motion, a.mjcf), indent=2) + "\n")


if __name__ == "__main__":
    main()
