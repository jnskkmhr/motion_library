"""Build portable dataset and browser catalogs from the motion folders."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LABELS = {
    "claw": "CLAW",
    "omniretarget": "OmniRetarget",
    "parc": "PARC",
    "deep_wholebody_parkour": "Deep Whole-body Parkour",
    "kimodo": "Kimodo",
}
NAMES = {
    "00452_crouch": "Crouched walking",
    "00543_hand_crawl": "Hand crawl",
    "02067_elbow_crawl": "Elbow crawl",
    "jump_001": "Running jump 01",
    "jump_002": "Running jumps 02",
    "jump_003": "Running jumps 03",
    "jump_004": "Turning jumps 04",
    "jump_005": "Running jumps 05",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    clips = []
    for source, label in LABELS.items():
        for path in sorted((root / "data" / source).rglob("motion.npz")):
            folder = path.parent
            relative = folder.relative_to(root / "data" / source)
            name = NAMES.get(str(relative), str(relative).replace("_", " ").replace("/", " / ").capitalize())
            variants = []
            for stem, variant_name in [("motion", "Reference"), ("motion_generated", "Before contact refinement")]:
                npz, video = folder / f"{stem}.npz", folder / f"{stem}.mp4"
                if not npz.is_file():
                    continue
                if not video.is_file():
                    raise FileNotFoundError(f"Missing paired video: {video}")
                with np.load(npz, allow_pickle=False) as data:
                    fps = float(np.asarray(data["fps"]).item())
                    frames = len(data["joint_pos"])
                    asset = str(data["robot_asset"].item()) if "robot_asset" in data else "unspecified"
                    terrain = (
                        "mesh"
                        if "terrain_vertices" in data
                        else "boxes"
                        if "terrain_position" in data or "terrain_boxes" in data
                        else "ground"
                    )
                variants.append(
                    {
                        "name": variant_name,
                        "npz": str(npz.relative_to(root)),
                        "video": str(video.relative_to(root)),
                        "fps": fps,
                        "frames": frames,
                        "duration": frames / fps,
                        "last_sample_time": (frames - 1) / fps,
                        "sha256": hashlib.sha256(npz.read_bytes()).hexdigest(),
                        "asset": asset,
                        "terrain": terrain,
                    }
                )
            reports = {p.stem: str(p.relative_to(root)) for p in folder.glob("*.json")}
            poster = folder / "preview.jpg"
            clips.append(
                {
                    "id": str(folder.relative_to(root / "data")),
                    "source": source,
                    "source_name": label,
                    "name": name,
                    "experimental": source == "kimodo",
                    "variants": variants,
                    "reports": reports,
                    "poster": str(poster.relative_to(root)) if poster.is_file() else None,
                }
            )
    catalog = {"schema_version": 1, "clips": clips}
    encoded = json.dumps(catalog, indent=2, ensure_ascii=False) + "\n"
    (root / "data/catalog.json").write_text(encoded)
    (root / "docs/catalog.js").write_text("window.MOTION_CATALOG = " + encoded.rstrip() + ";\n")
    print(f"Catalog: {len(clips)} clips; {sum(len(c['variants']) for c in clips)} video/NPZ pairs")


if __name__ == "__main__":
    main()
