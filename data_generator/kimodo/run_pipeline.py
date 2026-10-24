# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Run Kimodo generation, shared ball-hand G1 export, optional terrain fitting and replay.

Run with uv. USD export and playback use --isaaclab_python; model generation
uses --kimodo_python.
"""

from __future__ import annotations

import argparse
import json
import math
import shlex
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--prompt", help="Generate new motion with Kimodo-G1-RP-v1.")
    source.add_argument("--example", type=Path, help="Import a supplied Kimodo G1 NPZ without model generation.")
    source.add_argument(
        "--platform", action="store_true", help="Generate a four-second step onto a fixed low platform."
    )
    parser.add_argument("--platform_height", type=float, default=0.15)
    parser.add_argument("--robot_mjcf", type=Path, help="Ball-hand G1 source MJCF, required with --platform.")
    parser.add_argument(
        "--kimodo_python", type=Path, required=True, help="Python executable in the Kimodo environment."
    )
    parser.add_argument("--output", type=Path, required=True, help="New output directory, usually data/kimodo/<name>.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=44)
    parser.add_argument("--constraints", type=Path)
    parser.add_argument("--example_fps", type=float, default=30.0)
    parser.add_argument("--terrain", choices=("none", "stairs", "overhead"), default="none")
    parser.add_argument("--video", action="store_true", help="Record a full headless Isaac Lab replay.")
    parser.add_argument(
        "--dry_run", action="store_true", help="Print commands without writing files or loading models."
    )
    parser.add_argument(
        "--isaaclab_python", type=Path, required=True, help="Python executable in the Isaac Lab environment."
    )
    args = parser.parse_args()
    if not args.isaaclab_python.is_file():
        parser.error("--isaaclab_python must point to an existing Isaac Lab interpreter.")
    if not math.isfinite(args.duration) or args.duration <= 0 or args.seed < 0:
        parser.error("Duration must be positive and finite; seed must be nonnegative.")
    if args.example and args.constraints:
        parser.error("--constraints applies only to --prompt.")
    if args.platform:
        if args.constraints or args.terrain != "none":
            parser.error("--platform authors its own constraints and fixed terrain; omit --constraints and --terrain.")
        if args.robot_mjcf is None or not args.robot_mjcf.is_file():
            parser.error("--platform requires --robot_mjcf pointing to the ball-hand source asset.")
        if not math.isfinite(args.platform_height) or not 0.05 <= args.platform_height <= 0.2:
            parser.error("Platform height must be between 0.05 and 0.20 m.")
        args.duration = 4.0
        args.prompt = (
            "A person steps up onto a low platform with the left foot, brings the right foot onto it, and stands still."
        )
    if not args.kimodo_python.is_file():
        parser.error(f"Kimodo Python executable not found: {args.kimodo_python}")
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Output directory is not empty; choose a new directory to preserve existing motions.")
    here = Path(__file__).resolve().parent
    shared = here.parent / "shared"
    viewer = here.parent.parent / "integrations/isaaclab/play_motion_file.py"
    output = args.output.resolve()
    if args.platform:
        args.constraints = output / "constraints.json"
    # Preserve the venv executable path; resolving its symlink selects the base interpreter.
    generate = [str(args.kimodo_python.absolute()), str(here / "generate.py"), "--output", str(output)]
    if args.example:
        generate += ["--example", str(args.example.resolve()), "--example_fps", str(args.example_fps)]
    else:
        generate += ["--prompt", args.prompt, "--duration", str(args.duration), "--seed", str(args.seed)]
        if args.constraints:
            generate += ["--constraints", str(args.constraints.resolve())]
    raw = output / ("motion.npz" if args.terrain == "none" else "motion_raw.npz")
    commands = [
        ("generate" if args.prompt else "import_example", generate),
        (
            "export_mimic",
            [
                str(args.isaaclab_python.absolute()),
                str(shared / "export_mimic.py"),
                "--input",
                str(output / "g1_qpos.npz"),
                "--output",
                str(raw),
                "--robot_asset",
                "UNITREE_G1_29DOF_BALL_HAND_CFG",
            ],
        ),
    ]
    if args.platform:
        platform_tool = [
            str(args.kimodo_python.absolute()),
            str(here / "terrain_platform.py"),
            "--output",
            str(output),
            "--mjcf",
            str(args.robot_mjcf.resolve()),
        ]
        commands.insert(
            0, ("prepare_platform", [*platform_tool, "--stage", "prepare", "--height", str(args.platform_height)])
        )
        commands.insert(2, ("refine_contacts", [*platform_tool, "--stage", "refine"]))
        export_command = commands[-1][1]
        export_command[export_command.index("--input") + 1] = str(output / "g1_refined_qpos.npz")
        generated_export = export_command.copy()
        generated_export[generated_export.index("--input") + 1] = str(output / "g1_terrain_qpos.npz")
        generated_export[generated_export.index("--output") + 1] = str(output / "motion_generated.npz")
        commands.insert(3, ("export_generated", generated_export))
        commands.append(("validate_platform", [*platform_tool, "--stage", "validate"]))
    if args.terrain != "none":
        fitter = "fit_stairs.py" if args.terrain == "stairs" else "fit_overhead.py"
        commands.append(
            (
                "fit_terrain",
                [
                    str(args.isaaclab_python.absolute()),
                    str(shared / fitter),
                    "--input",
                    str(raw),
                    "--output",
                    str(output / "motion.npz"),
                ],
            )
        )
    if args.video:
        camera = ["-1.2", "-2", "0.3"] if args.terrain == "overhead" else ["-2", "-3", "1.2"]
        if args.platform:
            camera = ["-1.2", "-2", "0.6"]
        commands.append(
            (
                "record_video",
                [
                    "uv",
                    "run",
                    "--no-project",
                    "--python",
                    str(args.isaaclab_python.absolute()),
                    "--with",
                    "imageio",
                    "--with",
                    "imageio-ffmpeg",
                    "python",
                    str(viewer),
                    "--motion_file",
                    str(output / "motion.npz"),
                    "--video",
                    "--headless",
                    "--video_folder",
                    str(output),
                    "--camera_offset",
                    *camera,
                ],
            )
        )
    status = {
        "robot_asset": "UNITREE_G1_29DOF_BALL_HAND_CFG",
        "terrain": "fixed_platform" if args.platform else args.terrain,
        "completed": [],
    }
    for stage, command in commands:
        print(f"[{stage}] {shlex.join(command)}", flush=True)
        if args.dry_run:
            continue
        output.mkdir(parents=True, exist_ok=True)
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError:
            status["failed_stage"] = stage
            (output / "pipeline_status.json").write_text(json.dumps(status, indent=2) + "\n")
            raise
        status["completed"].append(stage)
        if stage == "validate_platform":
            validation = json.loads((output / "validation.json").read_text())
            status["passes_geometric_checks"] = validation["refined"]["passes_geometric_checks"]
            status["dynamic_tracking_validated"] = False
        (output / "pipeline_status.json").write_text(json.dumps(status, indent=2) + "\n")


if __name__ == "__main__":
    main()
