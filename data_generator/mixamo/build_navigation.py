"""Download selected navigation motions, retarget to G1, and build video previews.

Final motion arrays and videos are published together. Every NPZ includes fitted terrain, and
its video is rendered from that same NPZ. Dynamic tracking remains unvalidated.
"""

import argparse
import concurrent.futures
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from huggingface_hub import hf_hub_download

from data_generator.build_acp_library import run
from data_generator.mixamo.quality import review

ROOT = Path(__file__).resolve().parents[2]


def build(item, manifest, args):
    # Raw retargeting inputs are temporary build artifacts, never active data.
    (ROOT / "outputs").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="mixamo_build_", dir=ROOT / "outputs") as work:
        return build_staged(item, manifest, args, Path(work))


def build_staged(item, manifest, args, work):
    name = item["name"]
    final_dest = ROOT / "data/mixamo" / name
    if final_dest.exists() and not args.resume:
        raise ValueError(f"Existing final output requires --resume: {final_dest}")
    dest = work / "raw"
    logs = ROOT / "outputs/mixamo_navigation_logs" / name
    logs.mkdir(parents=True, exist_ok=True)
    request = {
        "item": {key: value for key, value in item.items() if key != "terrain_kind"},
        "revision": manifest["revision"],
        "max_duration": manifest["max_duration_seconds"],
        "mjcf_sha256": hashlib.sha256(args.mjcf.read_bytes()).hexdigest(),
        "retarget_sha256": hashlib.sha256((Path(__file__).with_name("import_mixamo.py")).read_bytes()).hexdigest(),
    }
    marker = dest / "build_request.json"
    if dest.exists() and any(dest.iterdir()):
        if not args.resume or not marker.exists() or json.loads(marker.read_text()) != request:
            raise ValueError(f"Existing output differs or --resume not supplied: {dest}")
    else:
        dest.mkdir(parents=True, exist_ok=True)
    source = hf_hub_download(
        manifest["repo_id"],
        item["file"],
        repo_type="dataset",
        revision=manifest["revision"],
        local_dir=ROOT / "vendor/mixamo_navigation",
    )
    if not (dest / "g1_qpos.npz").exists():
        # The importer requires an empty output; write the request marker afterward.
        run(
            [
                args.mixamo_python,
                Path(__file__).with_name("import_mixamo.py"),
                "--input",
                source,
                "--mjcf",
                args.mjcf,
                "--output",
                dest,
                "--max_duration",
                manifest["max_duration_seconds"],
            ],
            logs / "import.log",
        )
    marker.write_text(json.dumps(request, indent=2) + "\n")
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
            logs / "export.log",
        )
    # Fit in an isolated staging folder. Promote NPZ and video together only
    # after fitting, collision checks, rendering, and timing checks succeed.
    raw_dest = dest
    dest = work / "paired"
    dest.mkdir(parents=True, exist_ok=True)
    terrain_request = {
        "kind": item["terrain_kind"],
        "raw_sha256": hashlib.sha256((raw_dest / "motion_raw.npz").read_bytes()).hexdigest(),
        "fitter_sha256": hashlib.sha256(Path(__file__).with_name("fit_terrain.py").read_bytes()).hexdigest(),
    }
    terrain_marker = dest / "terrain_request.json"
    if not terrain_marker.exists() or json.loads(terrain_marker.read_text()) != terrain_request:
        (dest / "motion.mp4").unlink(missing_ok=True)
        run(
            [
                args.isaaclab_python,
                "-m",
                "data_generator.mixamo.fit_terrain",
                "--input",
                raw_dest / "motion_raw.npz",
                "--output",
                dest / "motion.npz",
                "--kind",
                item["terrain_kind"],
            ],
            logs / "terrain_fit.log",
        )
        shutil.copy2(dest / "motion.json", dest / "terrain_fit.json")
        terrain_marker.write_text(json.dumps(terrain_request, indent=2) + "\n")
    run(
        [
            args.library_python,
            "-m",
            "data_generator.mixamo.validate_terrain",
            "--motion",
            dest / "motion.npz",
            "--mjcf",
            args.mjcf,
            "--output",
            dest / "terrain_validation.json",
        ],
        logs / "terrain_validation.log",
    )
    collision = json.loads((dest / "terrain_validation.json").read_text())
    if not collision["robot_terrain_clear_within_2mm"]:
        raise ValueError(f"Terrain intersects G1: {name}: {collision['worst_penetration']}")
    review(dest / "motion.npz", dest / "terrain_fit.json", dest / "quality_review.json")
    run(
        [
            args.library_python,
            ROOT / "tools/screen_motion.py",
            "--motion",
            dest / "motion.npz",
            "--mjcf",
            args.mjcf,
            "--output",
            dest / "validation.json",
        ],
        logs / "screen.log",
    )
    with np.load(dest / "motion.npz", allow_pickle=False) as data:
        fps, frames = float(data["fps"]), len(data["joint_pos"])
    if frames / fps > manifest["max_duration_seconds"] + 1e-6:
        raise ValueError(f"Clip exceeds duration cap: {name}")
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
            logs / "video.log",
        )
    probe = json.loads(
        subprocess.check_output(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=nb_frames,r_frame_rate",
                "-of",
                "json",
                str(dest / "motion.mp4"),
            ],
            text=True,
        )
    )["streams"][0]
    a, b = map(float, probe["r_frame_rate"].split("/"))
    if int(probe["nb_frames"]) != frames or abs(a / b - fps) > 1e-6:
        raise ValueError(f"Video timing mismatch: {name}")
    title = item["description"] or item["prompt"]
    for prefix in ("Male ", "Female "):
        title = title.removeprefix(prefix)
    source_url = f"https://huggingface.co/datasets/{manifest['repo_id']}/blob/{manifest['revision']}/{item['file']}"
    fit_report = json.loads((dest / "terrain_fit.json").read_text())
    description = (
        f"{item['category']}. Static {item['terrain_kind']} terrain fitted to this G1 reference and stored in the NPZ. "
        "Motion samples and timing preserved. Kinematic replay; dynamic tracking and support contacts remain unvalidated."
    )
    provenance = json.loads((raw_dest / "provenance.json").read_text())
    provenance.update(
        source_url=source_url,
        source_repository=manifest["repo_id"],
        source_revision=manifest["revision"],
        source_motion_id=item["motion_id"],
        source_description=item["description"],
        category=item["category"],
        display_name=title,
        description=description,
        terrain_missing=False,
        terrain_source="static geometry fitted to the target G1 reference; not supplied by Mixamo",
        terrain_kind=item["terrain_kind"],
        terrain_box_count=fit_report["terrain_box_count"],
        motion_sha256=fit_report["motion_sha256"],
        target_robot="UNITREE_G1_29DOF_BALL_HAND_CFG",
        public_video_only=False,
        public_distribution=True,
    )
    (dest / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    final_dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(raw_dest / "build_request.json", final_dest / "build_request.json")
    for filename in (
        "motion.npz",
        "motion.mp4",
        "provenance.json",
        "validation.json",
        "terrain_fit.json",
        "terrain_validation.json",
        "quality_review.json",
    ):
        shutil.copy2(dest / filename, final_dest / filename)
    preview = ROOT / "docs/previews" / f"mixamo_{name}"
    preview.mkdir(parents=True, exist_ok=True)
    for filename in (
        "motion.mp4",
        "provenance.json",
        "validation.json",
        "terrain_fit.json",
        "terrain_validation.json",
        "quality_review.json",
    ):
        shutil.copy2(dest / filename, preview / filename)
    relative = str(preview.relative_to(ROOT))
    clip = {
        "id": f"mixamo/{name}",
        "name": title,
        "source": "mixamo",
        "source_name": "Mixamo",
        "category": item["category"],
        "description": description,
        "experimental": True,
        "preview_only": False,
        "local_only": False,
        "source_url": source_url,
        "poster": None,
        "variants": [
            {
                "name": "Motion + terrain",
                "npz": str((final_dest / "motion.npz").relative_to(ROOT)),
                "sha256": fit_report["motion_sha256"],
                "video": f"{relative}/motion.mp4",
                "video_sha256": hashlib.sha256((preview / "motion.mp4").read_bytes()).hexdigest(),
                "fps": fps,
                "frames": frames,
                "duration": frames / fps,
                "last_sample_time": (frames - 1) / fps,
                "asset": "UNITREE_G1_29DOF_BALL_HAND_CFG",
                "terrain": "boxes",
            }
        ],
        "reports": {
            key: f"{relative}/{key}.json"
            for key in ("provenance", "validation", "terrain_fit", "terrain_validation", "quality_review")
        },
    }
    (preview / "clip.json").write_text(json.dumps(clip, indent=2) + "\n")
    return {"name": name, "category": item["category"], "duration": frames / fps}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", type=Path, default=Path(__file__).with_name("navigation_manifest.json"))
    p.add_argument("--isaaclab_python", type=Path, required=True)
    p.add_argument("--mjcf", type=Path, required=True)
    p.add_argument("--mixamo_python", type=Path, default=ROOT / ".venv-mixamo/bin/python")
    p.add_argument("--library_python", type=Path, default=ROOT / ".venv/bin/python")
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--limit", type=int, help="Process the first N selected clips for a smoke check")
    p.add_argument("--resume", action="store_true")
    args = p.parse_args()
    if args.workers < 1:
        p.error("Workers must be positive")
    for path in (args.isaaclab_python, args.mjcf, args.mixamo_python, args.library_python):
        if not path.is_file():
            p.error(f"Missing dependency: {path}")
    manifest = json.loads(args.manifest.read_text())
    jobs = manifest["clips"][: args.limit]
    summary = {"completed": [], "failed": []}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(build, item, manifest, args): item for item in jobs}
        for future in concurrent.futures.as_completed(futures):
            item = futures[future]
            try:
                result = future.result()
                summary["completed"].append(result)
                print(f"Completed {len(summary['completed'])}/{len(jobs)}: {result}", flush=True)
            except Exception as exc:
                summary["failed"].append({"name": item["name"], "error": str(exc)})
                print(f"FAILED {item['name']}: {exc}", flush=True)
            path = ROOT / "outputs/mixamo_navigation_summary.json"
            path.write_text(json.dumps(summary, indent=2) + "\n")
    if summary["failed"]:
        raise SystemExit(f"{len(summary['failed'])} clips failed; inspect outputs/mixamo_navigation_logs")


if __name__ == "__main__":
    main()
