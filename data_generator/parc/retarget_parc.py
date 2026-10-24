# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Retarget PARC motion-terrain pairs to ball-hand G1 with calibrated Mink IK.

This baseline scales motion and terrain together to the robot's leg length.
It preserves source timing and fits root, torso and legs with Mink. Arm IK
uses anatomical segment directions in the torso frame, with neutral wrists.
It is not OmniRetarget or a physics rollout. Requires mink and daqp.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import pickle
import xml.etree.ElementTree as ET
from pathlib import Path

import mink
import mujoco
import numpy as np
from data_generator.deep_wholebody_parkour.import_deep_parkour import load_robot
from scipy.ndimage import label
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class ArrayUnpickler(pickle.Unpickler):
    """Read the published array/dict format without importing arbitrary classes."""

    def find_class(self, module, name):
        modules = (
            "numpy",
            "numpy.core.multiarray",
            "numpy._core.multiarray",
            "numpy.core.numeric",
            "numpy._core.numeric",
        )
        if module in modules and name in ("ndarray", "dtype", "_reconstruct", "scalar", "_frombuffer"):
            return super().find_class(module, name)
        raise ValueError(f"Unsupported pickle class: {module}.{name}")


def load_pair(path: Path):
    """Decode motion and terrain, leaving unused miscellaneous metadata opaque."""
    container = ArrayUnpickler(io.BytesIO(path.read_bytes())).load()
    return tuple(ArrayUnpickler(io.BytesIO(container[k])).load() for k in ("motion_data", "terrain_data"))


def source_fk(xml: Path, motion: dict):
    """Evaluate PARC's 15-body skeleton with local xyzw joint rotations."""
    root = ET.parse(xml).getroot().find("worldbody/body")
    bodies = list(root.iter("body"))
    names = [b.get("name") for b in bodies]
    parents = {child.get("name"): body.get("name") for body in bodies for child in body.findall("body")}
    count = len(motion["root_pos"])
    positions = np.zeros((count, len(bodies), 3))
    rotations = np.zeros((count, len(bodies), 4))
    positions[:, 0] = motion["root_pos"]
    rotations[:, 0] = motion["root_rot"]
    if motion["joint_rot"].shape != (count, len(bodies) - 1, 4):
        raise ValueError("Unexpected PARC skeleton")
    for i, body in enumerate(bodies[1:], 1):
        parent = names.index(parents[body.get("name")])
        parent_rot = Rotation.from_quat(rotations[:, parent])
        local_pos = np.fromstring(body.get("pos", "0 0 0"), sep=" ")
        local_rot = Rotation.from_quat(np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")[[1, 2, 3, 0]])
        positions[:, i] = positions[:, parent] + parent_rot.apply(local_pos)
        rotations[:, i] = (parent_rot * local_rot * Rotation.from_quat(motion["joint_rot"][:, i - 1])).as_quat()
    return names, positions, rotations


def terrain_boxes(terrain: dict, scale: float):
    """Merge equal-height grid cells into boxes using PARC's cell-center convention."""
    hf = np.asarray(terrain["hf"])
    dx = float(terrain["dx"])
    minimum = np.asarray(terrain["min_point"])
    used = np.zeros(hf.shape, bool)
    base = float(hf.min() - 0.1)
    boxes = []
    for i, j in np.ndindex(hf.shape):
        if used[i, j]:
            continue
        height = hf[i, j]
        end_j = j + 1
        while end_j < hf.shape[1] and not used[i, end_j] and abs(hf[i, end_j] - height) < 1e-6:
            end_j += 1
        end_i = i + 1
        while (
            end_i < hf.shape[0]
            and not used[end_i, j:end_j].any()
            and np.allclose(hf[end_i, j:end_j], height, atol=1e-6)
        ):
            end_i += 1
        used[i:end_i, j:end_j] = True
        center = minimum + np.array([(i + end_i - 1) / 2, (j + end_j - 1) / 2]) * dx
        boxes.append([*center, (height + base) / 2, (end_i - i) * dx, (end_j - j) * dx, height - base])
    return np.asarray(boxes) * scale, base * scale


def retarget_arms(model, result, names, source_pos, source_rot, source_fps):
    """Fit arm directions in the torso frame without moving the root or legs.

    G1's zero elbow coordinate is not a straight arm. Solve using actual link
    positions instead of equating human and robot joint coordinates or reference
    poses. PARC's fixed wrists map to neutral G1 wrists. The small temporal cost
    selects a continuous joint solution without prescribing the elbow bend.
    """
    data = mujoco.MjData(model)
    source_torso_inverse = Rotation.from_quat(source_rot[:, names.index("torso")]).inv()
    torso_id = model.body("torso_link").id
    report = {}
    for side in ("left", "right"):
        joint_ids = [
            model.joint(f"{side}_{part}_joint").id
            for part in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")
        ]
        addresses = model.jnt_qposadr[joint_ids]
        limits = model.jnt_range[joint_ids]
        wrist_addresses = [
            model.jnt_qposadr[model.joint(f"{side}_wrist_{axis}_joint").id] for axis in ("roll", "pitch", "yaw")
        ]
        body_ids = [model.body(f"{side}_{part}_link").id for part in ("shoulder_roll", "elbow", "wrist_roll")]
        source_points = np.stack(
            [source_pos[:, names.index(f"{side}_{part}")] for part in ("upper_arm", "lower_arm", "hand")], axis=1
        )
        directions = np.diff(source_points, axis=1)
        directions /= np.linalg.norm(directions, axis=-1, keepdims=True)
        local_directions = np.stack([source_torso_inverse.apply(directions[:, j]) for j in range(2)], axis=1)
        source_wrist_rotation = Rotation.from_quat(
            source_rot[:, names.index(f"{side}_lower_arm")]
        ).inv() * Rotation.from_quat(source_rot[:, names.index(f"{side}_hand")])
        if np.max(source_wrist_rotation.magnitude()) > 1e-5:
            raise ValueError("Arm conversion expects PARC's fixed-wrist skeleton.")
        previous = np.array([0.0, 0.0, 0.0, 0.2])
        direction_errors, flexion_errors, limit_frames = [], [], []
        for frame, pose in enumerate(result):
            data.qpos[:] = pose
            data.qpos[wrist_addresses] = 0.0
            mujoco.mj_kinematics(model, data)
            desired = Rotation.from_matrix(data.xmat[torso_id].reshape(3, 3)).apply(local_directions[frame])

            def residual(angles):
                data.qpos[addresses] = angles
                mujoco.mj_kinematics(model, data)
                segments = np.diff(data.xpos[body_ids], axis=0)
                segments /= np.linalg.norm(segments, axis=-1, keepdims=True)
                return np.r_[(segments - desired).ravel(), 0.03 * (angles - previous)]

            lower, upper = limits[:, 0].copy(), limits[:, 1].copy()
            if frame > 0:
                # Avoid shoulder branch jumps near a singularity. Leave headroom
                # below G1's 37 rad/s arm limit for subsequent PCHIP resampling.
                max_step = 20.0 / source_fps
                lower = np.maximum(lower, previous - max_step)
                upper = np.minimum(upper, previous + max_step)
            solution = least_squares(
                residual,
                np.clip(previous, lower + 1e-6, upper - 1e-6),
                bounds=(lower, upper),
                max_nfev=60,
                ftol=1e-9,
                xtol=1e-9,
                gtol=1e-9,
            )
            if not solution.success or not np.isfinite(solution.x).all():
                raise ValueError(f"Arm IK did not converge: {side}, frame {frame}: {solution.message}")
            residual(solution.x)
            actual = np.diff(data.xpos[body_ids], axis=0)
            actual /= np.linalg.norm(actual, axis=-1, keepdims=True)
            direction_errors.append(np.rad2deg(np.arccos(np.clip(np.sum(actual * desired, axis=-1), -1, 1))))
            source_bend = np.arccos(np.clip(desired[0] @ desired[1], -1, 1))
            target_bend = np.arccos(np.clip(actual[0] @ actual[1], -1, 1))
            flexion_errors.append(float(np.rad2deg(abs(source_bend - target_bend))))
            limit_frames.append(bool(np.min(abs(solution.x[-1] - limits[-1])) < 1e-4))
            pose[addresses] = solution.x
            pose[wrist_addresses] = 0.0
            previous = solution.x
        report[side] = {
            "mean_elbow_flexion_error_deg": float(np.mean(flexion_errors)),
            "max_elbow_flexion_error_deg": float(np.max(flexion_errors)),
            "max_segment_direction_error_deg": float(np.max(direction_errors)),
            "elbow_limit_frame_fraction": float(np.mean(limit_frames)),
        }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", type=Path, required=True)
    parser.add_argument("--source_xml", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    motion, terrain = load_pair(args.motion)
    names, source_pos, source_rot = source_fk(args.source_xml, motion)
    neutral_motion = {
        "root_pos": np.zeros((1, 3)),
        "root_rot": np.array([[0, 0, 0, 1]]),
        "joint_rot": np.tile([0, 0, 0, 1], (1, 14, 1)),
    }
    _, neutral_pos, neutral_rot = source_fk(args.source_xml, neutral_motion)
    model = load_robot(args.mjcf)
    data = mujoco.MjData(model)
    neutral = model.qpos0.copy()
    data.qpos[:] = neutral
    mujoco.mj_kinematics(model, data)
    foot_geom = model.geom("left_foot_collision").id
    foot_bottom = (
        data.geom_xpos[foot_geom, 2] - np.abs(data.geom_xmat[foot_geom].reshape(3, 3)[2]) @ model.geom_size[foot_geom]
    )
    source_leg = -neutral_pos[0, names.index("left_foot"), 2] + 0.05
    scale = (neutral[2] - foot_bottom) / source_leg
    pairs = {"pelvis": "pelvis", "torso_link": "torso"}
    for side in ("left", "right"):
        pairs.update(
            {
                f"{side}_hip_yaw_link": f"{side}_thigh",
                f"{side}_knee_link": f"{side}_shin",
                f"{side}_ankle_roll_link": f"{side}_foot",
            }
        )
    tasks, targets = [], []
    for target, source in pairs.items():
        body, index = model.body(target).id, names.index(source)
        src_neutral_rot = Rotation.from_quat(neutral_rot[0, index])
        offset_rotation = src_neutral_rot.inv() * Rotation.from_quat(data.xquat[body], scalar_first=True)
        offset_position = src_neutral_rot.inv().apply(data.xpos[body] - neutral[:3] - neutral_pos[0, index] * scale)
        orientations = Rotation.from_quat(source_rot[:, index])
        positions = source_pos[:, index] * scale + orientations.apply(offset_position)
        rotations = (orientations * offset_rotation).as_quat(scalar_first=True)
        tasks.append(
            mink.FrameTask(
                frame_name=target,
                frame_type="body",
                position_cost=80 if "ankle" in target else (40 if target == "pelvis" else 8),
                orientation_cost=8 if target in ("pelvis", "torso_link") else 2,
                lm_damping=1,
            )
        )
        targets.append((positions, rotations))
    configuration = mink.Configuration(model)
    configuration.update(neutral)
    posture = mink.PostureTask(model, cost=0.02)
    posture.set_target(neutral)
    limits = [mink.ConfigurationLimit(model)]
    dt = 1 / motion["fps"]
    result = []
    for frame in range(len(source_pos)):
        for task, (positions, rotations) in zip(tasks, targets, strict=True):
            task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rotations[frame]), positions[frame]))
        for _ in range(40 if frame == 0 else 10):
            velocity = mink.solve_ik(configuration, [*tasks, posture], dt, "daqp", damping=0.05, limits=limits)
            configuration.integrate_inplace(velocity, dt)
        result.append(configuration.q.copy())
    result = np.asarray(result)
    arm_report = retarget_arms(model, result, names, source_pos, source_rot, motion["fps"])
    boxes, ground = terrain_boxes(terrain, scale)
    joint_names = [model.joint(i).name for i in range(1, model.njnt)]
    feet = [names.index(s + "_foot") for s in ("left", "right")]
    contacts = np.asarray(motion["body_contacts"])[:, feet] > 0.5
    regions, count = label(~contacts.any(axis=1))
    flights = []
    for region in range(1, count + 1):
        ids = np.flatnonzero(regions == region)
        if len(ids) < 4 or ids[0] == 0 or ids[-1] == len(result) - 1:
            continue
        start, end = ids[0] - 1, ids[-1] + 1
        flights.append(
            {
                "start_s": start * dt,
                "end_s": end * dt,
                "pelvis_horizontal_travel_m": float(np.linalg.norm(result[end, :2] - result[start, :2])),
            }
        )
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output / "g1_qpos.npz",
        qpos=np.c_[result[:, :3], result[:, [4, 5, 6, 3]], result[:, 7:]],
        fps=motion["fps"],
        joint_names=joint_names,
        quaternion_order="xyzw",
        terrain_boxes=boxes,
        terrain_box_quat=np.tile([0, 0, 0, 1], (len(boxes), 1)),
        ground_height=ground,
    )
    np.savez_compressed(
        args.output / "source_contacts.npz", body_names=names, body_contacts=motion["body_contacts"], fps=motion["fps"]
    )
    report = {
        "source": "https://huggingface.co/datasets/mxucg/PARC",
        "archive_entry": "dec_release/initial_aug/jumping/" + args.motion.name.removeprefix("terrain_"),
        "source_sha256": hashlib.sha256(args.motion.read_bytes()).hexdigest(),
        "source_motion": str(args.motion),
        "processing": (
            "Mink root/torso/leg IK; joint-limited arm IK matching source segment directions in the torso frame; "
            "neutral fixed wrists; original timing; uniform motion and terrain scale"
        ),
        "retarget_version": "parc_anatomical_arms_v2",
        "arm_alignment": arm_report,
        "arm_source_frame_speed_cap_rad_s": 20.0,
        "arm_temporal_residual_weight": 0.03,
        "arm_angle_definition": "0 degrees is straight; shoulder_roll -> elbow -> wrist_roll link origins",
        "retarget_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "uniform_scale": float(scale),
        "terrain_modified": "Uniformly scaled with motion; no terrain fitting",
        "fps": motion["fps"],
        "frames": len(result),
        "source_contact_flight_intervals": flights,
        "dynamic_tracking_validated": False,
        "target_contacts_validated": False,
    }
    (args.output / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
