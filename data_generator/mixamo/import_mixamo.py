"""Import a local Mixamo FBX and retarget its body landmarks to G1.

Requires Python 3.11 with `uv sync --extra mixamo`. This is a kinematic IK baseline, not ACP's
retargeter. No model training or public redistribution of Mixamo motion is implied.
"""

import argparse
import hashlib
import json
from pathlib import Path

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from data_generator.deep_wholebody_parkour.import_deep_parkour import load_robot
from data_generator.parc.retarget_parc import retarget_arms

BONES = {
    "pelvis": "Hips",
    "torso": "Spine2",
    "left_thigh": "LeftUpLeg",
    "left_shin": "LeftLeg",
    "left_foot": "LeftFoot",
    "right_thigh": "RightUpLeg",
    "right_shin": "RightLeg",
    "right_foot": "RightFoot",
    "left_upper_arm": "LeftArm",
    "left_lower_arm": "LeftForeArm",
    "left_hand": "LeftHand",
    "right_upper_arm": "RightArm",
    "right_lower_arm": "RightForeArm",
    "right_hand": "RightHand",
}


def extract_fbx(path, fps, stack_index):
    try:
        import bpy
    except ImportError as exc:
        raise RuntimeError(
            "FBX import requires Blender's Python 3.11 wheel. Run with "
            "UV_PROJECT_ENVIRONMENT=.venv-mixamo uv run --python 3.11 --extra mixamo."
        ) from exc

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.render.fps = max(1, round(fps))
    scene.render.fps_base = scene.render.fps / fps
    bpy.ops.import_scene.fbx(
        filepath=str(path.resolve()), use_anim=True, use_image_search=False, automatic_bone_orientation=False
    )
    rigs = [obj for obj in scene.objects if obj.type == "ARMATURE"]
    if len(rigs) != 1:
        raise ValueError("Expected exactly one animated armature in the FBX.")
    rig = rigs[0]
    mapping = {}
    for bone in rig.data.bones:
        name = bone.name.rsplit(":", 1)[-1]
        if name in BONES.values():
            if name in mapping:
                raise ValueError(f"Ambiguous skeleton bone: {name}")
            mapping[name] = bone.name
    missing = set(BONES.values()) - mapping.keys()
    if missing:
        raise ValueError(f"Expected Mixamo skeleton; missing bones: {sorted(missing)}")
    actions = list(bpy.data.actions)
    if not actions or not 0 <= stack_index < len(actions):
        raise ValueError("Select a valid FBX animation with --animation_index.")
    action = actions[stack_index]
    rig.animation_data_create()
    rig.animation_data.action = action
    first, last = action.frame_range
    if last <= first:
        raise ValueError("FBX animation has no positive duration.")
    bone_names = [mapping[name] for name in BONES.values()]

    def unpack(matrices):
        positions = np.asarray([matrix.translation[:] for matrix in matrices])
        rotations = np.asarray([tuple(matrix.to_quaternion()) for matrix in matrices])
        return positions, rotations[:, [1, 2, 3, 0]]

    rest_pos, rest_quat = unpack([rig.matrix_world @ rig.data.bones[name].matrix_local for name in bone_names])
    positions, quaternions = [], []
    for frame in first + np.arange(int(np.floor(last - first + 1e-5)) + 1):
        scene.frame_set(int(np.floor(frame)), subframe=float(frame % 1))
        matrices = [rig.matrix_world @ rig.pose.bones[name].matrix for name in bone_names]
        pos, quat = unpack(matrices)
        positions.append(pos)
        quaternions.append(quat)
    return list(BONES), np.asarray(positions), np.asarray(quaternions), rest_pos, rest_quat, action.name


def unit(vector):
    length = np.linalg.norm(vector, axis=-1, keepdims=True)
    if np.any(length < 1e-6):
        raise ValueError("Degenerate human skeleton segment.")
    return vector / length


def body_frame(positions, names):
    up = unit(positions[..., names.index("torso"), :] - positions[..., names.index("pelvis"), :])
    left = positions[..., names.index("left_thigh"), :] - positions[..., names.index("right_thigh"), :]
    left = unit(left - np.sum(left * up, axis=-1, keepdims=True) * up)
    forward = unit(np.cross(left, up))
    return Rotation.from_matrix(np.stack([forward, left, up], axis=-1))


def retarget(names, positions, quaternions, rest_pos, rest_quat, fps, mjcf):
    model = load_robot(mjcf)
    data = mujoco.MjData(model)
    neutral = model.qpos0.copy()
    data.qpos[:] = neutral
    mujoco.mj_kinematics(model, data)
    root_id, torso_id = model.body("pelvis").id, model.body("torso_link").id
    source_neutral = body_frame(rest_pos, names)
    align = Rotation.from_matrix(data.xmat[root_id].reshape(3, 3)) * source_neutral.inv()
    positions = align.apply(positions.reshape(-1, 3)).reshape(positions.shape)
    rest_pos = align.apply(rest_pos)
    quaternions = (align * Rotation.from_quat(quaternions.reshape(-1, 4))).as_quat().reshape(quaternions.shape)
    rest_quat = (align * Rotation.from_quat(rest_quat)).as_quat()
    root_rot = body_frame(positions, names)
    target_leg, source_leg = [], []
    for side in ("left", "right"):
        target = [model.body(f"{side}_{part}_link").id for part in ("hip_yaw", "knee", "ankle_roll")]
        source = [names.index(f"{side}_{part}") for part in ("thigh", "shin", "foot")]
        target_leg.append(np.linalg.norm(np.diff(data.xpos[target], axis=0), axis=1).sum())
        source_leg.append(np.linalg.norm(np.diff(rest_pos[source], axis=0), axis=1).sum())
    scale = float(np.mean(target_leg) / np.mean(source_leg))
    root = positions[:, names.index("pelvis")] * scale
    root[:, :2] -= root[0, :2].copy()
    root[:, 2] += neutral[2] - rest_pos[names.index("pelvis"), 2] * scale
    targets = {
        "pelvis": (root, root_rot),
        "torso_link": (root + root_rot.apply(data.xpos[torso_id] - data.xpos[root_id]), root_rot),
    }
    neutral_root_rot = Rotation.from_matrix(data.xmat[root_id].reshape(3, 3))
    for side in ("left", "right"):
        hip, knee, ankle = [model.body(f"{side}_{part}_link").id for part in ("hip_yaw", "knee", "ankle_roll")]
        human_hip, human_knee, human_ankle = [
            positions[:, names.index(f"{side}_{part}")] for part in ("thigh", "shin", "foot")
        ]
        hip_pos = root + root_rot.apply(neutral_root_rot.inv().apply(data.xpos[hip] - data.xpos[root_id]))
        knee_pos = hip_pos + unit(human_knee - human_hip) * np.linalg.norm(data.xpos[knee] - data.xpos[hip])
        ankle_pos = knee_pos + unit(human_ankle - human_knee) * np.linalg.norm(data.xpos[ankle] - data.xpos[knee])
        source_index = names.index(f"{side}_foot")
        correction = Rotation.from_quat(rest_quat[source_index]).inv() * Rotation.from_matrix(
            data.xmat[ankle].reshape(3, 3)
        )
        foot_rot = Rotation.from_quat(quaternions[:, source_index]) * correction
        targets[f"{side}_knee_link"] = (knee_pos, root_rot)
        targets[f"{side}_ankle_roll_link"] = (ankle_pos, foot_rot)
    configuration = mink.Configuration(model)
    configuration.update(neutral)
    tasks = {
        name: mink.FrameTask(
            frame_name=name,
            frame_type="body",
            position_cost=80 if "ankle" in name else 30,
            orientation_cost=0 if "knee" in name else 5,
            lm_damping=1,
        )
        for name in targets
    }
    posture = mink.PostureTask(model, cost=0.02)
    posture.set_target(neutral)
    limits = [mink.ConfigurationLimit(model)]
    dt = 1 / fps
    result = []
    for i in range(len(root)):
        for name, (pos, rot) in targets.items():
            tasks[name].set_target(
                mink.SE3.from_rotation_and_translation(mink.SO3(rot[i].as_quat(scalar_first=True)), pos[i])
            )
        for _ in range(40 if i == 0 else 10):
            velocity = mink.solve_ik(configuration, [*tasks.values(), posture], dt, "daqp", damping=0.05, limits=limits)
            configuration.integrate_inplace(velocity, dt)
        result.append(configuration.q.copy())
    result = np.asarray(result)
    # Reuse anatomical arm fitting. Wrist motion is intentionally not transferred.
    arm_rot = quaternions.copy()
    arm_rot[:, names.index("torso")] = root_rot.as_quat()
    for side in ("left", "right"):
        arm_rot[:, names.index(f"{side}_hand")] = arm_rot[:, names.index(f"{side}_lower_arm")]
    arms = retarget_arms(model, result, names, positions, arm_rot, fps)
    return model, result, {"uniform_scale": scale, "arm_alignment": arms}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True, help="Local Mixamo FBX with skeleton and animation")
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--fps", type=float, default=30)
    p.add_argument("--animation_index", type=int, default=0)
    p.add_argument("--max_duration", type=float, default=60.0, help="Maximum output duration in seconds (default 60)")
    p.add_argument(
        "--terrain", type=Path, help="Optional JSON with explicit canonical box/mesh fields, in target G1 meters/Z-up"
    )
    a = p.parse_args()
    if not a.input.is_file() or a.input.suffix.lower() != ".fbx":
        p.error("Provide an existing .fbx file.")
    if not np.isfinite(a.fps) or a.fps <= 0:
        p.error("FPS must be finite and positive.")
    if a.output.exists() and any(a.output.iterdir()):
        p.error("Choose a new output directory.")
    if not np.isfinite(a.max_duration) or a.max_duration <= 0:
        p.error("Maximum duration must be finite and positive.")
    names, pos, quat, rest_pos, rest_quat, animation = extract_fbx(a.input, a.fps, a.animation_index)
    source_frames = len(pos)
    # Leave room for the last resampled frame at the 50 Hz export rate.
    cap = int(np.floor((a.max_duration - 1 / 50) * a.fps)) + 1
    pos, quat = pos[:cap], quat[:cap]
    if len(pos) < 3:
        p.error("Animation needs at least three sampled frames.")
    model, result, report = retarget(names, pos, quat, rest_pos, rest_quat, a.fps, a.mjcf)
    terrain = {"ground_height": 0.0}
    if a.terrain:
        terrain = json.loads(a.terrain.read_text())
        allowed = {
            "terrain_position",
            "terrain_dimension",
            "terrain_quat",
            "terrain_vertices",
            "terrain_faces",
            "ground_height",
        }
        if not set(terrain) <= allowed:
            p.error("Terrain JSON contains unsupported fields.")
        if "terrain_position" in terrain:
            terrain["terrain_boxes"] = np.c_[terrain.pop("terrain_position"), terrain.pop("terrain_dimension")]
        if "terrain_quat" in terrain:
            terrain["terrain_box_quat"] = terrain.pop("terrain_quat")
    a.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        a.output / "g1_qpos.npz",
        qpos=np.c_[result[:, :3], result[:, [4, 5, 6, 3]], result[:, 7:]],
        fps=a.fps,
        joint_names=[model.joint(i).name for i in range(1, model.njnt)],
        quaternion_order="xyzw",
        **terrain,
    )
    report.update(
        source="Mixamo local FBX",
        source_file=a.input.name,
        source_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
        animation=animation,
        fps=a.fps,
        frames=len(result),
        source_frames=source_frames,
        max_duration_seconds=a.max_duration,
        truncated=len(result) < source_frames,
        public_distribution=False,
        exact_acp_clip=False,
        terrain_source=str(a.terrain) if a.terrain else "Assumed ground Z=0; obstacle geometry not recovered",
        source_joint_trajectory_modified=True,
        wrists="Fixed neutral; source wrist motion omitted",
        dynamic_tracking_validated=False,
        ready_for_training=False,
        limitations="Baseline IK only; verify contact geometry and source bind pose before tracking.",
    )
    (a.output / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
