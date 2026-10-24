# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate, refine, export and record a terrain-first Kimodo batch with a review gallery.

Run with uv and provide --isaaclab_python for USD export and playback. --process_existing resumes processing an
already generated batch, reusing completed files. Use a new output for new settings.
"""

from __future__ import annotations

import argparse
import html
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


def run_stage(command: list[str], log: Path):
    """Run one stage with a persistent diagnostic log."""
    with log.open("w") as stream:
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)


def gallery(output: Path, status: list[dict]):
    """Write inspectable results without treating geometric validity as dynamic validity."""
    cards = []
    selected = []
    for item in status:
        directory = item["directory"]
        dest = output / directory
        if item["status"] == "failed":
            cards.append(
                f"<article><h2>{html.escape(directory)}</h2>"
                f"<p>Processing failed: {html.escape(item['error'])}</p></article>"
            )
            continue
        report = json.loads((dest / "validation.json").read_text())
        refined, generated = report["refined"], report["generated"]
        ok = refined["passes_geometric_checks"]
        if ok:
            selected.append(str((dest / "motion.npz").resolve()))
        rows = "".join(
            f"<tr><td>{label}</td><td>{generated[key] * scale:.2f}</td><td>{refined[key] * scale:.2f}</td></tr>"
            for label, key, scale in [
                ("Maximum robot–terrain penetration (mm)", "max_robot_terrain_penetration_m", 1000),
                ("Maximum joint-limit violation (rad)", "max_joint_limit_violation_rad", 1),
                ("Forward travel (m)", "forward_travel_m", 1),
            ]
        )
        video = "".join(
            f'<div><h3>{label}</h3><video controls loop preload="metadata" src="{directory}/{name}.mp4"></video></div>'
            for name, label in [("motion_generated", "Kimodo output"), ("motion", "After contact refinement")]
            if (dest / f"{name}.mp4").exists()
        )
        reasons = "; ".join(refined["rejection_reasons"]) or "Passed sampled geometry and contact checks."
        cards.append(
            f"<article><h2>{html.escape(directory.replace('_', ' '))}</h2>"
            f'<p class="{"pass" if ok else "fail"}">{html.escape(reasons)}</p><div class="videos">{video}</div>'
            f"<table><tr><th>Measurement</th><th>Generated</th><th>Refined</th></tr>{rows}</table>"
            f'<p><a href="{directory}/motion.npz">Mimic NPZ</a> · '
            f'<a href="{directory}/terrain.json">Terrain</a> · '
            f'<a href="{directory}/validation.json">Validation</a> · '
            f'<a href="{directory}/refinement.json">Corrections</a></p></article>'
        )
    (output / "geometrically_valid.json").write_text(
        json.dumps(
            {
                "dynamic_tracking_validated": False,
                "motion_files": selected,
                "note": (
                    "Sampled kinematic checks only; configure matching terrain and robot in the training environment."
                ),
            },
            indent=2,
        )
        + "\n"
    )
    page = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Terrain-first G1 motion batch</title><style>
body{font:16px/1.6 system-ui;margin:0;padding:28px;background:#111b24;color:#e8edf2}main{max-width:1400px;margin:auto}
article{background:#1d2b37;padding:22px;margin:24px 0;border-radius:12px}
h1{font-size:32px}h2{font-size:23px}h3{font-size:17px}
.videos{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:20px}video{width:100%;border-radius:8px}
a,.pass{color:#83d8bd}.fail{color:#ffc584}table{width:100%;border-collapse:collapse}
td,th{text-align:left;border-bottom:1px solid #40515e;padding:8px}
</style><main><h1>Terrain-first G1 motion batch</h1>
<p>Fixed scenes → sparse pose constraints → Kimodo → contact refinement at 50 Hz → ball-hand G1 USDA playback.</p>
<p>These are kinematic references, not tracking-policy rollouts.
Contact intervals are estimated from motion geometry and speed, with prescribed jump support windows.
Passing geometric checks does not establish balance, actuator feasibility, or freedom from robot self-collision.</p>
<p>Scene dimensions are stored in each clip's terrain.json.
The terrain was specified before generation and remains unchanged.</p>"""
    page += (
        f"<p>{len(status)} candidates; {len(selected)} pass geometric checks. "
        '<a href="geometrically_valid.json">Filtered list</a></p>'
    )
    (output / "index.html").write_text(page + "".join(cards) + "</main></html>")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kimodo_python", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("terrain_batch.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--process_existing", action="store_true")
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--video", action="store_true")
    parser.add_argument(
        "--isaaclab_python", type=Path, required=True, help="Python executable in the Isaac Lab environment."
    )
    args = parser.parse_args()
    if not args.isaaclab_python.is_file():
        parser.error("--isaaclab_python must point to an existing Isaac Lab interpreter.")
    if args.workers < 1 or args.workers > 4:
        parser.error("Use one to four independent processing workers.")
    here = Path(__file__).resolve().parent
    helper = [
        str(args.kimodo_python.absolute()),
        str(here / "terrain_batch.py"),
        "--output",
        str(args.output.resolve()),
        "--mjcf",
        str(args.mjcf.resolve()),
    ]
    if not args.process_existing:
        if args.output.exists() and any(args.output.iterdir()):
            parser.error("Use a new output directory or --process_existing.")
        args.output.mkdir(parents=True, exist_ok=True)
        # Preparation rejects nonempty output, so place its log outside that directory.
        run_stage(
            [*helper, "--stage", "prepare", "--manifest", str(args.manifest.resolve())],
            args.output.with_suffix(".prepare.log"),
        )
        run_stage(
            [*helper, "--stage", "generate", "--manifest", str(args.manifest.resolve())], args.output / "generate.log"
        )
    manifest = json.loads((args.output / "requests.json").read_text())
    helper += ["--manifest", str((args.output / "requests.json").resolve())]
    jobs = [
        {"name": motion["name"], "seed": seed, "directory": f"{motion['name']}/seed_{seed}", "status": "pending"}
        for motion in manifest["motions"]
        for seed in manifest["seeds"]
    ]

    def process(item):
        dest = args.output / item["directory"]
        case_helper = [*helper, "--only", item["name"], "--seeds", str(item["seed"])]
        if not (dest / "g1_refined_qpos.npz").exists():
            run_stage([*case_helper, "--stage", "refine"], dest / "refine.log")
        for source, target in [("g1_terrain_qpos.npz", "motion_generated"), ("g1_refined_qpos.npz", "motion")]:
            if not (dest / f"{target}.npz").exists():
                run_stage(
                    [
                        str(args.isaaclab_python.absolute()),
                        str(here.parent / "shared/export_mimic.py"),
                        "--input",
                        str(dest / source),
                        "--output",
                        str(dest / f"{target}.npz"),
                        "--robot_asset",
                        "UNITREE_G1_29DOF_BALL_HAND_CFG",
                    ],
                    dest / f"{target}.export.log",
                )
        run_stage([*case_helper, "--stage", "validate"], dest / "validate.log")
        if args.video:
            offset = ["-1", "-2", "0.3"] if item["name"] in ["crawl", "bear_walk"] else ["-1.4", "-2.3", "0.7"]
            for name in ["motion", "motion_generated"]:
                if not (dest / f"{name}.mp4").exists():
                    run_stage(
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
                            str(here.parent.parent / "integrations/isaaclab/play_motion_file.py"),
                            "--motion_file",
                            str(dest / f"{name}.npz"),
                            "--video",
                            "--headless",
                            "--video_folder",
                            str(dest),
                            "--camera_offset",
                            *offset,
                        ],
                        dest / f"{name}.replay.log",
                    )
        report = json.loads((dest / "validation.json").read_text())
        return "geometric_pass" if report["refined"]["passes_geometric_checks"] else "needs_refinement"

    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        pending = {executor.submit(process, item): item for item in jobs}
        for future in as_completed(pending):
            item = pending[future]
            try:
                item["status"] = future.result()
            except Exception as error:
                item.update(status="failed", error=str(error))
            print(item["directory"], item["status"], flush=True)
            (args.output / "batch_status.json").write_text(json.dumps(jobs, indent=2) + "\n")
    gallery(args.output, jobs)
    if any(item["status"] == "failed" for item in jobs):
        raise RuntimeError("Some processing stages failed; see batch_status.json and per-clip logs.")


if __name__ == "__main__":
    main()
