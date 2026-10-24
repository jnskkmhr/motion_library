# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Convert released Deep Whole-body Parkour references from torso-rooted G1 to pelvis-rooted G1.

Preserve source timing and terrain. Reversing the waist chain requires negating
its joint angles and computing the pelvis pose from the source URDF. No tracking
policy is executed by this importer. Run with numpy, scipy, trimesh and mujoco.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def load_robot(path: Path) -> mujoco.MjModel:
    """Load the target MJCF with resolved mesh paths and its required floor."""
    xml = ET.parse(path).getroot()
    compiler = xml.find("compiler")
    compiler.set("meshdir", str((path.parent / compiler.get("meshdir", ".")).resolve()))
    ET.SubElement(xml.find("worldbody"), "geom", name="floor", type="plane", size="0 0 .1")
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))


def source_fk(urdf: Path, names: list[str], angles: np.ndarray, position: np.ndarray, quaternion: np.ndarray):
    """Evaluate the source URDF tree with torso poses in wxyz quaternion order."""
    transforms = np.tile(np.eye(4), (len(angles), 1, 1))
    transforms[:, :3, :3] = Rotation.from_quat(quaternion[:, [1, 2, 3, 0]]).as_matrix()
    transforms[:, :3, 3] = position
    world = {"torso_link": transforms}
    pending = list(ET.parse(urdf).getroot().findall("joint"))
    while pending:
        resolved = []
        for joint in pending:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in world:
                continue
            origin = joint.find("origin")
            translation = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            local = np.tile(np.eye(4), (len(angles), 1, 1))
            local[:, :3, 3] = translation
            rotation = Rotation.from_euler("xyz", rpy)
            if joint.get("type") == "revolute":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                rotation = rotation * Rotation.from_rotvec(angles[:, names.index(joint.get("name")), None] * axis)
            elif joint.get("type") != "fixed":
                raise ValueError(f"Unsupported source joint: {joint.get('name')}")
            local[:, :3, :3] = rotation.as_matrix()
            world[child] = world[parent] @ local
            resolved.append(joint)
        if not resolved:
            raise ValueError("Expected a connected source URDF rooted at torso_link")
        pending = [j for j in pending if j not in resolved]
    return world


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", type=Path, required=True)
    parser.add_argument("--terrain", type=Path, required=True)
    parser.add_argument("--source_urdf", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with np.load(args.motion, allow_pickle=False) as src:
        names = src["joint_names"].tolist()
        angles = src["joint_pos"].astype(np.float64)
        fps = float(src["framerate"])
        world = source_fk(args.source_urdf, names, angles, src["base_pos_w"], src["base_quat_w"])
    pelvis = world["pelvis"]
    converted = angles.copy()
    for name in ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"):
        converted[:, names.index(name)] *= -1
    qpos = np.c_[pelvis[:, :3, 3], Rotation.from_matrix(pelvis[:, :3, :3]).as_quat(), converted]
    model = load_robot(args.mjcf)
    data = mujoco.MjData(model)
    target_names = [model.joint(i).name for i in range(1, model.njnt)]
    if set(names) != set(target_names) or not np.isfinite(qpos).all() or fps <= 0:
        raise ValueError("Invalid G1 motion")
    order = [names.index(n) for n in target_names]
    measured = [
        "pelvis",
        "torso_link",
        "left_ankle_roll_link",
        "right_ankle_roll_link",
        "left_wrist_yaw_link",
        "right_wrist_yaw_link",
    ]
    errors = {n: [] for n in measured}
    for frame in qpos:
        data.qpos[:] = np.r_[frame[:3], frame[[6, 3, 4, 5]], frame[7:][order]]
        mujoco.mj_kinematics(model, data)
        for name in measured:
            errors[name].append(data.xpos[model.body(name).id].copy())
    errors = {n: float(np.linalg.norm(np.asarray(v) - world[n][:, :3, 3], axis=-1).max()) for n, v in errors.items()}
    if errors["pelvis"] > 1e-6 or max(errors[n] for n in measured if "ankle" in n) > 0.001:
        raise ValueError(f"Source/target pelvis or foot FK mismatch: {errors}")
    mesh = trimesh.load_mesh(args.terrain)
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output / "g1_qpos.npz",
        qpos=qpos,
        fps=fps,
        joint_names=names,
        quaternion_order="xyzw",
        terrain_vertices=np.asarray(mesh.vertices),
        terrain_faces=np.asarray(mesh.faces),
        ground_height=float(mesh.bounds[0, 2] - 0.02),
    )
    report = {
        "source": "https://project-instinct.github.io/deep-whole-body-parkour/",
        "source_motion": str(args.motion),
        "source_sha256": hashlib.sha256(args.motion.read_bytes()).hexdigest(),
        "source_urdf_sha256": hashlib.sha256(args.source_urdf.read_bytes()).hexdigest(),
        "source_root": "torso_link",
        "source_quaternion_order": "wxyz",
        "target_root": "pelvis",
        "processing": "Source URDF FK to pelvis; negate reversed waist joints; original timing and terrain retained",
        "terrain_modified": False,
        "fps": fps,
        "frames": len(qpos),
        "max_source_target_link_position_difference_m": errors,
        "dynamic_tracking_validated": False,
    }
    (args.output / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
