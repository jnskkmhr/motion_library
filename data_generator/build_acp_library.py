"""Import available ACP source datasets, export exact-asset G1 NPZs, and record previews."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

from data_generator.lafan.import_lafan import import_clip
from data_generator.omniretarget.download import download

ROOT = Path(__file__).resolve().parents[1]


def run(command, log):
    with log.open("w") as stream:
        subprocess.run([str(c) for c in command], stdout=stream, stderr=subprocess.STDOUT, check=True, cwd=ROOT)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--isaaclab_python", type=Path, required=True)
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--sources", nargs="+", choices=["omniretarget", "lafan"], default=["omniretarget", "lafan"])
    p.add_argument("--manifest", type=Path, default=Path(__file__).with_name("acp_sources.json"))
    p.add_argument("--omniretarget_source", type=Path, help="Use an already extracted pinned dataset")
    p.add_argument("--resume", action="store_true", help="Continue the same manifest, retaining completed stages")
    p.add_argument(
        "--output_root",
        type=Path,
        default=ROOT / "data",
        help="Destination root; use a new folder to reproduce bundled clips",
    )
    args = p.parse_args()
    if not args.isaaclab_python.is_file() or not args.mjcf.is_file():
        p.error("Python interpreter and G1 MJCF must exist.")
    manifest = json.loads(args.manifest.read_text())
    jobs = []
    if "omniretarget" in args.sources:
        source = args.omniretarget_source or download(ROOT / "vendor/omniretarget")
        jobs += [("omniretarget", name, source) for name in manifest["omniretarget"]["clips"]]
    if "lafan" in args.sources:
        jobs += [("lafan", item["name"], item) for item in manifest["lafan"]]
    for source, name, item in jobs:
        dest = args.output_root.resolve() / source / name
        if dest.exists() and any(dest.iterdir()) and not args.resume:
            raise FileExistsError(f"Output exists: {dest}; use --resume for unchanged requests.")
        dest.mkdir(parents=True, exist_ok=True)
        logdir = ROOT / "outputs/acp_logs" / source / name
        logdir.mkdir(parents=True, exist_ok=True)
        request = {
            "source": source,
            "name": name,
            "settings": item if isinstance(item, dict) else str(item.resolve()),
            "mjcf": str(args.mjcf.resolve()),
        }
        request_file = dest / "build_request.json"
        if request_file.exists() and json.loads(request_file.read_text()) != request:
            raise ValueError(f"Resume request changed: {dest}")
        # Importers author the first files before the request marker is written.
        if not (dest / "g1_qpos.npz").exists():
            if source == "lafan":
                import_clip(item["clip"], item["start"], item["end"], dest)
            else:
                run(
                    [
                        sys.executable,
                        ROOT / "data_generator/omniretarget/import_omniretarget.py",
                        "--source",
                        item,
                        "--clips",
                        name,
                        "--output",
                        dest.parent,
                    ],
                    logdir / "import.log",
                )
        request_file.write_text(json.dumps(request, indent=2) + "\n")
        if not (dest / "motion.npz").exists():
            if not (dest / "motion_raw.npz").exists():
                run(
                    [
                        args.isaaclab_python,
                        ROOT / "data_generator/shared/export_mimic.py",
                        "--input",
                        dest / "g1_qpos.npz",
                        "--output",
                        dest / "motion_raw.npz",
                        "--robot_asset",
                        "UNITREE_G1_29DOF_BALL_HAND_CFG",
                    ],
                    logdir / "export.log",
                )
            run(
                [
                    sys.executable,
                    ROOT / "tools/normalize_terrain.py",
                    "--input",
                    dest / "motion_raw.npz",
                    "--output",
                    dest / "motion.npz",
                    "--ground_height",
                    "0",
                ],
                logdir / "normalize.log",
            )
        run(
            [
                sys.executable,
                ROOT / "tools/screen_motion.py",
                "--motion",
                dest / "motion.npz",
                "--mjcf",
                args.mjcf,
                "--output",
                dest / "validation.json",
            ],
            logdir / "screen.log",
        )
        if not (dest / "motion.mp4").exists():
            run(
                [
                    "uv",
                    "run",
                    "--no-project",
                    "--python",
                    args.isaaclab_python,
                    "--with",
                    "imageio",
                    "--with",
                    "imageio-ffmpeg",
                    "python",
                    ROOT / "integrations/isaaclab/play_motion_file.py",
                    "--motion_file",
                    dest / "motion.npz",
                    "--video",
                    "--headless",
                    "--video_folder",
                    dest,
                    "--video_resolution",
                    "960",
                    "640",
                    "--camera_offset",
                    "-1.5",
                    "-2.5",
                    "0.8",
                ],
                logdir / "video.log",
            )
        provenance = json.loads((dest / "provenance.json").read_text())
        provenance.update(
            exact_acp_clip=False, target_robot="UNITREE_G1_29DOF_BALL_HAND_CFG", dynamic_tracking_validated=False
        )
        (dest / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        print(f"Completed {source}/{name}", flush=True)


if __name__ == "__main__":
    main()
