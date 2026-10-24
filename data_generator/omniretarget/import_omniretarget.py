# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Import public OmniRetarget G1 trajectories and their supplied terrain geometry.

This imports existing retargeted motion; it does not generate new motion or infer
terrain from motion. Mesh boxes are recovered with their original orientation.
Source coordinate ordering was checked against Drake's GetPositionNames for the
published g1_29dof_spherehand.urdf at the pinned dataset revision.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

DATASET_REVISION = "135259130158f1cb1799e4ab9b6566c938285fc9"


def origin_transform(origin: ET.Element | None) -> np.ndarray:
    """Read a URDF origin as a homogeneous transform."""
    transform = np.eye(4)
    if origin is not None:
        transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        transform[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return transform


def read_terrain(urdf_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Recover box centers, full sizes and orientations from supplied box meshes."""
    root = ET.parse(urdf_path).getroot()
    transforms = {"world": np.eye(4)}
    joints = list(root.findall("joint"))
    while joints:
        resolved = []
        for joint in joints:
            if joint.get("type") != "fixed":
                raise ValueError("Terrain import only supports fixed joints.")
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent in transforms:
                transforms[child] = transforms[parent] @ origin_transform(joint.find("origin"))
                resolved.append(joint)
        if not resolved:
            raise ValueError("Terrain links must form a fixed tree rooted at world.")
        joints = [joint for joint in joints if joint not in resolved]
    boxes, quaternions = [], []
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            mesh_node = collision.find("geometry/mesh")
            mesh = trimesh.load_mesh(urdf_path.parent / mesh_node.get("filename"), process=False)
            mesh.vertices *= np.fromstring(mesh_node.get("scale", "1 1 1"), sep=" ")
            mesh.apply_transform(transforms[link.get("name")] @ origin_transform(collision.find("origin")))
            world_to_box, size = trimesh.bounds.oriented_bounds(mesh)
            local = trimesh.transform_points(mesh.vertices, world_to_box)
            if not np.allclose(np.abs(local), size / 2, atol=1e-5):
                raise ValueError(f"Terrain geometry is not a box: {urdf_path}")
            box_to_world = np.linalg.inv(world_to_box)
            boxes.append(np.r_[box_to_world[:3, 3], size])
            quaternions.append(Rotation.from_matrix(box_to_world[:3, :3]).as_quat())
    if not boxes:
        raise ValueError(f"No terrain boxes found in {urdf_path}")
    return np.asarray(boxes), np.asarray(quaternions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Extracted OmniRetarget dataset root.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--clips", nargs="+", required=True, help="Clip IDs, e.g. climb_14 climb_26.")
    args = parser.parse_args()
    robot_path = args.source / "models/g1/g1_29dof_spherehand.urdf"
    joint_names = [
        joint.get("name") for joint in ET.parse(robot_path).findall("joint") if joint.get("type") == "revolute"
    ]
    if len(joint_names) != 29 or len(set(joint_names)) != 29:
        raise ValueError("Expected the published G1 29DOF source robot.")
    for clip in args.clips:
        if len(clip) != 8 or not clip.startswith("climb_") or not clip[6:].isdigit():
            parser.error(f"Invalid clip ID: {clip}")
        source_path = args.source / f"robot-terrain/{clip}_z_scale_1.0.npz"
        terrain_path = args.source / f"models/terrain/{clip}/multi_boxes_z_scale_1.0.urdf"
        with np.load(source_path, allow_pickle=False) as data:
            qpos, fps = data["qpos"], float(data["fps"])
        if qpos.ndim != 2 or qpos.shape[1] != 36 or not np.isfinite(qpos).all() or fps <= 0:
            raise ValueError(f"Invalid robot-only motion: {source_path}")
        # Drake [qw,qx,qy,qz,x,y,z,joints] -> exporter [x,y,z,qx,qy,qz,qw,joints].
        converted = np.concatenate((qpos[:, 4:7], qpos[:, 1:4], qpos[:, :1], qpos[:, 7:]), axis=-1)
        boxes, quats = read_terrain(terrain_path)
        destination = args.output / clip
        destination.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            destination / "g1_qpos.npz",
            qpos=converted,
            fps=fps,
            joint_names=joint_names,
            quaternion_order="xyzw",
            terrain_boxes=boxes,
            terrain_box_quat=quats,
        )
        metadata = {
            "source_type": "public_retargeted_motion",
            "dataset": "omniretarget/OmniRetarget_Dataset",
            "revision": DATASET_REVISION,
            "license": "MIT (dataset card)",
            "source_motion": str(source_path),
            "source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
            "terrain_source": "supplied paired terrain; original dimensions and transforms",
            "terrain_urdf": str(terrain_path),
            "frames": len(qpos),
            "fps": fps,
            "source_robot": "g1_29dof_spherehand.urdf",
            "target_robot": "UNITREE_G1_29DOF_BOX_FOOT_CFG",
            "contact_labels": "not supplied; no labels fabricated",
            "dynamic_tracking_validated": False,
        }
        (destination / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(f"Imported {clip}: {len(qpos)} frames, {fps} Hz, {len(boxes)} terrain boxes")


if __name__ == "__main__":
    main()
