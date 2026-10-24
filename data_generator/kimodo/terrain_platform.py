# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Author a fixed platform and Kimodo constraints, then refine G1 contacts with IK.

This proof of concept uses a prescribed left-then-right step and support schedule.
Run in the Kimodo environment with mujoco installed. Geometry never follows motion.
The MJCF must be the source of the ball-hand USDA used by the shared exporter.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp


class PlatformKinematics:
    """Own the exact G1 MJCF and mutable FK state for a fixed platform scene."""

    def __init__(self, mjcf: Path, scene: dict):
        xml = ET.parse(mjcf).getroot()
        compiler = xml.find("compiler")
        compiler.set("meshdir", str((mjcf.parent / compiler.get("meshdir", ".")).resolve()))
        world = xml.find("worldbody")
        ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 .1")
        surface_names = ["floor"]
        for index, (box, quaternion) in enumerate(zip(scene["terrain_boxes"], scene["terrain_box_quat"], strict=True)):
            box = np.asarray(box)
            name = "platform" if index == 0 else f"terrain_{index}"
            surface_names.append(name)
            ET.SubElement(
                world,
                "geom",
                name=name,
                type="box",
                pos=" ".join(map(str, box[:3])),
                size=" ".join(map(str, box[3:] / 2)),
                quat=" ".join(map(str, np.asarray(quaternion)[[3, 0, 1, 2]])),
            )
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.names = [self.model.joint(i).name for i in range(1, self.model.njnt)]
        if self.model.nq != 36 or len(self.names) != 29:
            raise ValueError("Expected a floating-base 29-joint G1.")
        self.feet = [self.model.body(f"{side}_ankle_roll_link").id for side in ("left", "right")]
        self.foot_geoms = [self.model.geom(f"{side}_foot_collision").id for side in ("left", "right")]
        self.surfaces = [self.model.geom(name).id for name in surface_names]
        self.collision_geoms = [
            i for i in range(self.model.ngeom) if self.model.geom_bodyid[i] != 0 and self.model.geom_contype[i] != 0
        ]
        self.legs = [i + 7 for i, name in enumerate(self.names) if any(x in name for x in ("hip", "knee", "ankle"))]
        self.variables = [0, 1, 2, *self.legs]
        self.limits = self.model.jnt_range[1:]
        self.lower = np.r_[[-2, -1, 0.35], self.limits[np.array(self.legs) - 7, 0]]
        self.upper = np.r_[[3, 1, 2], self.limits[np.array(self.legs) - 7, 1]]
        self.sole_offsets = []
        for geom in self.foot_geoms:
            # Flat foot: ankle height above the bottom face of its collision box.
            self.sole_offsets.append(float(self.model.geom_size[geom, 2] - self.model.geom_pos[geom, 2]))

    def forward(self, qpos: np.ndarray):
        """Evaluate FK without advancing physics; qpos uses MuJoCo wxyz."""
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)

    def solve(self, reference: np.ndarray, targets: np.ndarray, active: np.ndarray, previous=None):
        """Refine base translation and leg joints; keep generated upper-body motion."""
        qpos = reference.copy()
        qpos[7:] = np.clip(qpos[7:], self.limits[:, 0], self.limits[:, 1])
        original = qpos[self.variables].copy()
        previous_delta = np.zeros(len(original)) if previous is None else previous

        def residual(values):
            qpos[self.variables] = values
            self.forward(qpos)
            position = (self.data.xpos[self.feet] - targets) * active[:, None]
            orientation = Rotation.from_matrix(self.data.xmat[self.feet].reshape(2, 3, 3)).as_rotvec()
            distances = [
                mujoco.mj_geomDistance(self.model, self.data, foot, surface, 0.1, None)
                for foot in self.collision_geoms
                for surface in self.surfaces
            ]
            return np.r_[
                250 * position.ravel(),
                30 * (orientation * active[:, None]).ravel(),
                400 * np.minimum(distances, 0),
                np.r_[15, 15, 15, np.full(len(self.legs), 1.0)] * (values - original),
                2 * (values - original - previous_delta),
            ]

        result = least_squares(
            residual,
            np.clip(original + previous_delta, self.lower, self.upper),
            bounds=(self.lower, self.upper),
            max_nfev=100,
            ftol=1e-7,
        )
        qpos[self.variables] = result.x
        return qpos.copy(), result.x - original


def planned_feet(times: np.ndarray, height: float, sole_offsets: list[float]):
    """Prescribed support intervals and swing guide for this four-second step-up."""
    positions = np.zeros((len(times), 2, 3))
    contacts = np.ones((len(times), 2), dtype=bool)
    for side, (start, end, final_x) in enumerate(((0.6, 1.6, 0.48), (2.0, 3.2, 0.60))):
        u = np.clip((times - start) / (end - start), 0, 1)
        smooth = u * u * (3 - 2 * u)
        positions[:, side, 0] = final_x * smooth
        positions[:, side, 1] = 0.12 if side == 0 else -0.12
        positions[:, side, 2] = height * smooth + sole_offsets[side] + 0.18 * np.sin(np.pi * u) ** 2
        contacts[:, side] = (times <= start) | (times >= end)
    return positions, contacts


def prepare(output: Path, mjcf: Path, height: float):
    """Write geometry and sparse pose constraints before loading the motion model."""
    from kimodo.exports.mujoco import MujocoQposConverter
    from kimodo.skeleton import G1Skeleton34

    scene = {
        "coordinate_system": "Z-up, X-forward, meters",
        "geometry_source": "specified_before_generation",
        "terrain_boxes": [[0.85, 0, height / 2, 1.2, 1.2, height]],
        "terrain_box_quat": [[0, 0, 0, 1]],
        "duration": 4.0,
        "fps": 30,
        "prompt": (
            "A person steps up onto a low platform with the left foot, brings the right foot onto it, and stands still."
        ),
        "contact_schedule": "Left swing 0.6–1.6 s; right swing 2.0–3.2 s; stationary support otherwise.",
        "mjcf": str(mjcf.resolve()),
        "mjcf_sha256": hashlib.sha256(mjcf.read_bytes()).hexdigest(),
    }
    robot = PlatformKinematics(mjcf, scene)
    times = np.arange(120) / 30
    targets, contacts = planned_feet(times, height, robot.sole_offsets)
    reference = robot.model.qpos0.copy()
    reference[:3] = [0, 0, 0.75]
    reference[3:7] = [1, 0, 0, 0]
    for i, name in enumerate(robot.names):
        for pattern, angle in (("hip_pitch", -0.312), ("knee", 0.669), ("ankle_pitch", -0.357), ("elbow", 0.6)):
            if pattern in name:
                reference[7 + i] = angle
    frames = np.array([0, 12, 33, 48, 60, 78, 96, 119])
    poses = []
    for frame in frames:
        guess = reference.copy()
        guess[0] = targets[frame, :, 0].mean()
        guess[2] += height * np.clip((times[frame] - 1.0) / 2.2, 0, 1)
        pose, _ = robot.solve(guess, targets[frame], np.ones(2))
        robot.forward(pose)
        if np.max(np.linalg.norm(robot.data.xpos[robot.feet] - targets[frame], axis=-1)) > 0.005:
            raise ValueError(f"Terrain-derived pose at frame {frame} is not reachable within 5 mm.")
        poses.append(pose)
    converter = MujocoQposConverter(G1Skeleton34())
    converter_names = [j.attrib["name"] for j in ET.parse(converter.xml_path).find("worldbody").iter("joint")]
    poses = np.asarray(poses)
    poses[:, 7:] = poses[:, [7 + robot.names.index(n) for n in converter_names]]
    # Repeat each sparse pose to satisfy the converter's temporal smoothing window.
    converted = converter.qpos_to_motion_dict(np.repeat(poses, 15, axis=0).astype(np.float32), 30)
    local = np.asarray(converted["local_rot_mats"])[::15]
    roots = np.asarray(converted["root_positions"])[::15]
    rotvec = Rotation.from_matrix(local.reshape(-1, 3, 3)).as_rotvec().reshape(-1, 34, 3)
    constraints = []
    for kind, indices in (("fullbody", [0, 7]), ("left-foot", list(range(1, 7))), ("right-foot", list(range(1, 7)))):
        constraints.append(
            {
                "type": kind,
                "frame_indices": frames[indices].tolist(),
                "local_joints_rot": rotvec[indices].tolist(),
                "root_positions": roots[indices].tolist(),
                "smooth_root_2d": roots[indices][:, [0, 2]].tolist(),
            }
        )
    output.mkdir(parents=True, exist_ok=True)
    for name, content in (("terrain.json", scene), ("constraints.json", constraints)):
        (output / name).write_text(json.dumps(content, indent=2) + "\n")
    np.savez_compressed(output / "contact_plan.npz", times=times, ankle_targets=targets, contacts=contacts)
    print(f"Prepared fixed {height:.3f} m platform and eight sparse constraint frames.", flush=True)


def refine(output: Path, mjcf: Path):
    """Refine generated support contacts while leaving the pre-authored terrain fixed."""
    scene = json.loads((output / "terrain.json").read_text())
    if hashlib.sha256(mjcf.read_bytes()).hexdigest() != scene["mjcf_sha256"]:
        raise ValueError("The robot source asset changed after constraint authoring.")
    robot = PlatformKinematics(mjcf, scene)
    with np.load(output / "g1_qpos.npz", allow_pickle=False) as source:
        data = {key: source[key] for key in source.files}
    if str(data["quaternion_order"].item()) != "xyzw":
        raise ValueError("Expected xyzw source qpos.")
    names = data["joint_names"].tolist()
    if set(names) != set(robot.names):
        raise ValueError("Source joints do not match the refinement asset.")
    source_qpos = data["qpos"]
    data["terrain_boxes"] = np.asarray(scene["terrain_boxes"])
    data["terrain_box_quat"] = np.asarray(scene["terrain_box_quat"])
    np.savez_compressed(output / "g1_terrain_qpos.npz", **data)
    poses = source_qpos.copy().astype(np.float64)
    poses[:, 3:7] = source_qpos[:, [6, 3, 4, 5]]
    poses[:, 7:] = source_qpos[:, [7 + names.index(n) for n in robot.names]]
    if len(poses) != 120 or float(data["fps"]) != 30:
        raise ValueError("This contact schedule requires a four-second, 30 Hz clip.")
    source_times = np.arange(len(poses)) / float(data["fps"])
    times = np.arange(int(np.floor(source_times[-1] * 50)) + 1) / 50
    # Refine the final playback samples: interpolation after IK can cross obstacle edges.
    rotations = Slerp(source_times, Rotation.from_quat(poses[:, [4, 5, 6, 3]]))(times).as_quat()
    poses = np.c_[
        CubicSpline(source_times, poses[:, :3])(times),
        rotations[:, [3, 0, 1, 2]],
        CubicSpline(source_times, poses[:, 7:])(times),
    ]
    targets, contacts = planned_feet(times, scene["terrain_boxes"][0][5], robot.sole_offsets)
    result, previous = [], None
    for frame, pose in enumerate(poses):
        corrected, previous = robot.solve(pose, targets[frame], contacts[frame].astype(float), previous)
        result.append(corrected)
    result = np.asarray(result)
    data["qpos"] = np.c_[result[:, :3], result[:, [4, 5, 6, 3]], result[:, [7 + robot.names.index(n) for n in names]]]
    data["terrain_boxes"] = np.asarray(scene["terrain_boxes"])
    data["terrain_box_quat"] = np.asarray(scene["terrain_box_quat"])
    # These are prescribed support labels, not Kimodo's flat-ground estimates.
    data["foot_contacts"] = np.repeat(contacts, 2, axis=1)
    data["fps"] = 50
    np.savez_compressed(output / "g1_refined_qpos.npz", **data)
    report = {
        "terrain_modified": False,
        "method": "Bounded per-frame IK for support feet and robot-terrain separation",
        "contact_schedule_source": "prescribed_before_generation",
        "refinement_fps": 50,
        "dynamic_tracking_validated": False,
        "max_root_correction_m": float(np.linalg.norm(result[:, :3] - poses[:, :3], axis=-1).max()),
        "max_joint_correction_rad": float(np.abs(result[:, 7:] - poses[:, 7:]).max()),
        "scope": (
            "Base translation and leg joints optimized; upper-body angles limited to MJCF ranges."
            " No dynamics optimization."
        ),
    }
    (output / "refinement.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


def validate(output: Path, mjcf: Path):
    """Measure the exported USD trajectory against fixed terrain and source MJCF FK."""
    scene = json.loads((output / "terrain.json").read_text())
    robot = PlatformKinematics(mjcf, scene)
    collision_geoms = [
        i for i in range(robot.model.ngeom) if robot.model.geom_bodyid[i] != 0 and robot.model.geom_contype[i] != 0
    ]
    report = {
        "terrain_modified": False,
        "dynamic_tracking_validated": False,
        "self_collision_validated": False,
        "collision_method": "MuJoCo geom distances; mesh convex hulls",
    }
    for label, filename in (("generated", "motion_generated.npz"), ("refined", "motion.npz")):
        with np.load(output / filename, allow_pickle=False) as data:
            for key in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"):
                if not np.isfinite(data[key]).all():
                    raise ValueError(f"Nonfinite {key} in {filename}.")
            if not np.allclose(data["terrain_boxes"], scene["terrain_boxes"], atol=1e-7):
                raise ValueError("Export changed the pre-authored terrain.")
            if not np.allclose(np.linalg.norm(data["body_quat_w"], axis=-1), 1, atol=1e-5):
                raise ValueError("Body quaternions are not normalized.")
            names = data["joint_names"].tolist()
            bodies = data["body_names"].tolist()
            root = bodies.index("pelvis")
            feet = [bodies.index(f"{side}_ankle_roll_link") for side in ("left", "right")]
            times = np.arange(len(data["joint_pos"])) / float(data["fps"])
            targets, contacts = planned_feet(times, scene["terrain_boxes"][0][5], robot.sole_offsets)
            errors = np.linalg.norm(data["body_pos_w"][:, feet] - targets, axis=-1)
            max_fk_error, min_distance = 0.0, 1.0
            worst_pair = None
            for frame in range(len(times)):
                qpos = np.r_[
                    data["body_pos_w"][frame, root],
                    data["body_quat_w"][frame, root][[3, 0, 1, 2]],
                    data["joint_pos"][frame, [names.index(n) for n in robot.names]],
                ]
                robot.forward(qpos)
                max_fk_error = max(
                    max_fk_error, float(np.max(np.abs(robot.data.xpos[robot.feet] - data["body_pos_w"][frame, feet])))
                )
                for geom in collision_geoms:
                    for surface in robot.surfaces:
                        distance = mujoco.mj_geomDistance(robot.model, robot.data, geom, surface, 0.1, None)
                        if distance < min_distance:
                            min_distance = distance
                            worst_pair = [robot.model.geom(geom).name, robot.model.geom(surface).name, frame]
            if max_fk_error > 1e-4:
                raise ValueError(f"MJCF and exported USDA ankle FK differ by {max_fk_error} m.")
            limits = data["joint_limits"]
            violation = float(np.maximum(limits[:, 0] - data["joint_pos"], data["joint_pos"] - limits[:, 1]).max())
            metrics = {
                "frames": len(times),
                "fps": float(data["fps"]),
                "max_support_ankle_target_error_m": float(errors[contacts].max()),
                "mean_support_ankle_target_error_m": float(errors[contacts].mean()),
                "max_robot_terrain_penetration_m": max(0, -float(min_distance)),
                "worst_pair": worst_pair,
                "max_joint_limit_violation_rad": max(0, violation),
                "max_mjcf_usda_ankle_fk_difference_m": max_fk_error,
                "max_joint_speed_rad_s": float(np.abs(data["joint_vel"]).max()),
                "max_joint_acceleration_rad_s2": float(
                    np.abs(np.gradient(data["joint_vel"], 1 / float(data["fps"]), axis=0)).max()
                ),
            }
            metrics["passes_geometric_checks"] = bool(
                errors[contacts].max() <= 0.01 and min_distance >= -0.002 and violation <= 1e-4
            )
            report[label] = metrics
    report["geometric_thresholds"] = {
        "support_error_m": 0.01,
        "terrain_penetration_m": 0.002,
        "joint_limit_violation_rad": 1e-4,
    }
    (output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("prepare", "refine", "validate"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--height", type=float, default=0.15)
    args = parser.parse_args()
    if not np.isfinite(args.height) or not 0.05 <= args.height <= 0.2:
        parser.error("This low-platform example supports heights from 0.05 to 0.20 m.")
    if args.stage == "prepare":
        if args.output.exists() and any(args.output.iterdir()):
            parser.error("Use an empty output directory to preserve previous motions.")
        prepare(args.output, args.mjcf, args.height)
    elif args.stage == "refine":
        refine(args.output, args.mjcf)
    else:
        validate(args.output, args.mjcf)


if __name__ == "__main__":
    main()
