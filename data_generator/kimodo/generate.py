# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate G1 motion with Kimodo, or explicitly import its public stair example.

Run in a separate environment with Kimodo installed. Outputs are Z-up, X-forward,
in meters and radians. The official G1 conversion replaces a SOMA/GMR retargeting
stage; this script does not claim to retarget human motion to arbitrary robots.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import torch
from kimodo import load_model
from kimodo.exports.mujoco import MujocoQposConverter
from kimodo.skeleton import G1Skeleton34, global_rots_to_local_rots
from kimodo.tools import seed_everything


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--prompt", help="Generate a new motion. Requires Kimodo's text encoder access.")
    source.add_argument("--example", type=Path, help="Import an existing Kimodo G1 NPZ; do not run generation.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=44)
    parser.add_argument("--constraints", type=Path, help="Optional Kimodo end-effector/keyframe constraints JSON.")
    parser.add_argument("--example_fps", type=float, default=30.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    metadata = {"model": "Kimodo-G1-RP-v1", "retargeting": "official Kimodo G1 joint projection; no SOMA/GMR"}
    if args.example:
        if args.constraints:
            parser.error("--constraints only applies to --prompt")
        with np.load(args.example, allow_pickle=False) as data:
            motion = {key: data[key] for key in data.files}
        skeleton = G1Skeleton34()
        fps = args.example_fps
        metadata.update(source_type="supplied_example", source_file=str(args.example.resolve()))
        metadata["source_sha256"] = hashlib.sha256(args.example.read_bytes()).hexdigest()
        meta_path = args.example.with_name("meta.json")
        if meta_path.exists():
            metadata["source_generation_settings"] = json.loads(meta_path.read_text())
    else:
        from kimodo.constraints import load_constraints_lst

        if args.duration <= 0:
            parser.error("--duration must be positive")
        model = load_model(metadata["model"], device="cuda")
        skeleton, fps = model.skeleton, model.fps
        seed_everything(args.seed)
        constraints = load_constraints_lst(str(args.constraints), skeleton) if args.constraints else []
        batch = model(
            [args.prompt],
            [round(args.duration * fps)],
            num_samples=1,
            num_denoising_steps=100,
            constraint_lst=constraints,
            post_processing=False,
            return_numpy=True,
        )
        motion = {key: value[0] for key, value in batch.items() if isinstance(value, np.ndarray)}
        metadata.update(source_type="generated", prompt=args.prompt, seed=args.seed, duration=args.duration)
        if args.constraints:
            metadata["constraints"] = json.loads(args.constraints.read_text())

    save_motion(motion, skeleton, fps, args.output, metadata)


def save_motion(motion: dict, skeleton: G1Skeleton34, fps: float, output: Path, metadata: dict):
    """Save one Kimodo G1 sample with named joint coordinates and provenance."""
    output.mkdir(parents=True, exist_ok=True)
    if fps <= 0 or motion["posed_joints"].shape[1] != 34:
        raise ValueError("Expected a positive frame rate and the 34-joint Kimodo G1 skeleton.")
    if "local_rot_mats" not in motion:
        motion["local_rot_mats"] = (
            global_rots_to_local_rots(torch.as_tensor(motion["global_rot_mats"]), skeleton).cpu().numpy()
        )
    if "root_positions" not in motion:
        motion["root_positions"] = motion["posed_joints"][:, skeleton.root_idx]
    converter = MujocoQposConverter(skeleton)
    qpos = converter.dict_to_qpos(motion, device="cpu", root_quat_w_first=False).reshape(-1, 36)
    joint_names = [joint.attrib["name"] for joint in ET.parse(converter.xml_path).find("worldbody").iter("joint")]
    if len(joint_names) != 29 or len(set(joint_names)) != 29:
        raise ValueError("Kimodo G1 XML must uniquely name 29 hinge joints.")
    # Save raw output so future projections or terrain fits can be reproduced.
    np.savez_compressed(output / "kimodo_raw.npz", **motion)
    np.savez_compressed(
        output / "g1_qpos.npz",
        qpos=qpos,
        fps=fps,
        joint_names=joint_names,
        quaternion_order="xyzw",
        foot_contacts=motion["foot_contacts"],
    )
    import kimodo

    repo = Path(kimodo.__file__).resolve().parents[1]
    revision = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"], capture_output=True, text=True)
    metadata.update(kimodo_revision=revision.stdout.strip() if revision.returncode == 0 else "unknown", fps=fps)
    (output / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Saved {len(qpos)} frames at {fps} Hz to {output / 'g1_qpos.npz'}")


if __name__ == "__main__":
    main()
