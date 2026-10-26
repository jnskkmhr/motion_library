"""Measure every saved G1 frame against the exact MJCF collision geometry and NPZ terrain."""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


def validate(motion, mjcf, output):
    with np.load(motion, allow_pickle=False) as archive:
        data = dict(archive)
    xml = ET.parse(mjcf).getroot()
    compiler = xml.find("compiler")
    compiler.set("meshdir", str((mjcf.parent / compiler.get("meshdir", ".")).resolve()))
    world = xml.find("worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="0 0 .1", pos=f"0 0 {float(data['ground_height'])}")
    surfaces = ["floor"]
    for i, (pos, dim, quat) in enumerate(
        zip(data["terrain_position"], data["terrain_dimension"], data["terrain_quat"], strict=True)
    ):
        name = f"paired_box_{i}"
        surfaces.append(name)
        ET.SubElement(
            world,
            "geom",
            name=name,
            type="box",
            pos=" ".join(map(str, pos)),
            size=" ".join(map(str, dim / 2)),
            quat=" ".join(map(str, quat[[3, 0, 1, 2]])),
        )
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    state = mujoco.MjData(model)
    robot = [i for i in range(model.ngeom) if model.geom_bodyid[i] != 0 and model.geom_contype[i] != 0]
    surface_ids = [model.geom(n).id for n in surfaces]
    names = data["joint_names"].tolist()
    body_names = data["body_names"].tolist()
    order = [names.index(model.joint(i).name) for i in range(1, model.njnt)]
    root = body_names.index("pelvis")
    worst = [0.0, None, None, None]
    counts = np.zeros(len(surface_ids), dtype=int)
    per_surface = np.full(len(surface_ids), np.inf)
    fk_error = 0.0
    common = [(i, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)) for i, n in enumerate(body_names)]
    common = [(i, j) for i, j in common if j >= 0]
    saved_ids, body_ids = zip(*common)
    for frame in range(len(data["joint_pos"])):
        state.qpos[:3] = data["body_pos_w"][frame, root]
        state.qpos[3:7] = data["body_quat_w"][frame, root, [3, 0, 1, 2]]
        state.qpos[7:] = data["joint_pos"][frame, order]
        mujoco.mj_kinematics(model, state)
        fk_error = max(
            fk_error,
            float(
                np.linalg.norm(state.xpos[list(body_ids)] - data["body_pos_w"][frame, list(saved_ids)], axis=-1).max()
            ),
        )
        for j, surface in enumerate(surface_ids):
            minimum = 0.2
            for geom in robot:
                if (
                    j > 0
                    and np.linalg.norm(state.geom_xpos[geom] - state.geom_xpos[surface])
                    > model.geom_rbound[geom] + model.geom_rbound[surface] + 0.2
                ):
                    continue
                gap = mujoco.mj_geomDistance(model, state, geom, surface, 0.2, None)
                minimum = min(minimum, gap)
                if gap < worst[0]:
                    worst = [float(gap), frame, model.geom(geom).name, surfaces[j]]
            per_surface[j] = min(per_surface[j], minimum)
            counts[j] += minimum < -0.002
    if fk_error > 0.002:
        raise ValueError(f"MJCF/USD body mismatch: {fk_error:.6f} m")
    report = {
        "motion_file": str(motion.name),
        "frames_checked": len(data["joint_pos"]),
        "collision_geometry": "all enabled G1 MJCF collision shapes, ground plane, and every NPZ terrain box",
        "max_mjcf_to_saved_usd_body_position_error_m": fk_error,
        "compared_body_names": [body_names[i] for i in saved_ids],
        "usd_fixed_bodies_absent_from_mjcf": [
            name for name in body_names if name not in [body_names[i] for i in saved_ids]
        ],
        "max_robot_terrain_penetration_m": -worst[0],
        "worst_penetration": dict(zip(("signed_distance_m", "frame", "robot_geom", "terrain_geom"), worst)),
        "terrain_minimum_signed_gaps_m": dict(zip(surfaces, per_surface.tolist())),
        "terrain_frames_with_penetration_over_2mm": dict(zip(surfaces, counts.tolist())),
        "robot_terrain_clear_within_2mm": worst[0] >= -0.002,
        "dynamic_tracking_validated": False,
        "support_contact_validated": False,
        "limitations": "Kinematic collision checks only; no self-collision, contact forces, balance, torque or tracking-policy test.",
    }
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"{motion.parent.name}: maximum robot-terrain penetration {-worst[0] * 1000:.2f} mm", flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--motion", type=Path, required=True)
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    validate(a.motion, a.mjcf, a.output)


if __name__ == "__main__":
    main()
