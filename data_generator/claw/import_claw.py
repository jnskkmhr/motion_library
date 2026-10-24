# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Import public CLAW G1 dynamic trajectories without inventing contact labels."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Downloaded CLAW dataset root.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--clips", nargs="+", required=True)
    args = parser.parse_args()
    model = ET.parse(args.source / "g1_xml/g1_29dof_rev_1_0.xml")
    joint_names = [joint.get("name") for joint in model.find("worldbody").iter("joint") if joint.get("type") != "free"]
    if len(joint_names) != 29 or len(set(joint_names)) != 29:
        raise ValueError("Expected 29 uniquely named hinge joints in the published G1 XML.")
    for clip in args.clips:
        if Path(clip).name != clip or clip in (".", ".."):
            parser.error("Clip names must be directory names within source/motion.")
        folder = args.source / "motion" / clip
        source_path = folder / "trajectory_dynamic.csv"
        annotation = json.loads((folder / "annotation.json").read_text())
        qpos = np.loadtxt(source_path, delimiter=",")
        if qpos.ndim != 2 or qpos.shape[1] != 36 or not np.isfinite(qpos).all():
            raise ValueError(f"Invalid G1 trajectory: {source_path}")
        output = args.output / clip
        output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            output / "g1_qpos.npz",
            qpos=qpos,
            fps=annotation["data_fps"],
            joint_names=joint_names,
            quaternion_order="xyzw",
        )
        metadata = {
            "source_type": "public_G1_dynamic_trajectory",
            "dataset": "JianuoCao/CLAW",
            "revision": "cbd516f611e46d52182d261ca1651563d8044850",
            "source_motion": str(source_path),
            "source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
            "annotation": annotation,
            "actual_frames": len(qpos),
            "fps": annotation["data_fps"],
            "terrain_source": "none supplied",
            "target_robot": "UNITREE_G1_29DOF_BOX_FOOT_CFG",
            "dynamic_tracking_validated_on_target_asset": False,
            "license": "No explicit dataset license in the downloaded card; retain upstream provenance.",
        }
        (output / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Imported {clip}: {len(qpos)} frames at {annotation['data_fps']} Hz")


if __name__ == "__main__":
    main()
