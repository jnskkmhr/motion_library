"""Check catalog links, named motion arrays, terrain geometry, and video timing."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=ROOT / "data/catalog.json")
    catalog = json.loads(parser.parse_args().catalog.read_text())
    require(shutil.which("ffprobe"), "Install FFmpeg (ffprobe) to validate video frame counts.")
    pairs = 0
    for clip in catalog["clips"]:
        for variant in clip["variants"]:
            path = ROOT / variant["npz"]
            require(hashlib.sha256(path.read_bytes()).hexdigest() == variant["sha256"], f"Checksum: {path}")
            with np.load(path, allow_pickle=False) as data:
                n = variant["frames"]
                require(n > 0 and variant["fps"] > 0, f"Invalid frame count or FPS: {path}")
                joints, bodies = len(data["joint_names"]), len(data["body_names"])
                require(joints == 29 and len(set(data["joint_names"])) == joints, f"Joint labels: {path}")
                require(len(set(data["body_names"])) == bodies, f"Body labels: {path}")
                for key, shape in [
                    ("joint_pos", (n, joints)),
                    ("joint_vel", (n, joints)),
                    ("body_pos_w", (n, bodies, 3)),
                    ("body_quat_w", (n, bodies, 4)),
                    ("body_lin_vel_w", (n, bodies, 3)),
                    ("body_ang_vel_w", (n, bodies, 3)),
                ]:
                    require(data[key].shape == shape and np.isfinite(data[key]).all(), f"{key}: {path}")
                require(
                    np.allclose(np.linalg.norm(data["body_quat_w"], axis=-1), 1, atol=1e-4), f"Quaternion norm: {path}"
                )
                require(str(data["quaternion_order"].item()) == "xyzw", f"Quaternion order: {path}")
                require("terrain_boxes" not in data and "terrain_box_quat" not in data, f"Legacy terrain: {path}")
                if "terrain_position" in data:
                    pos, dim = data["terrain_position"], data["terrain_dimension"]
                    require(pos.ndim == 2 and pos.shape[1] == 3 and pos.shape == dim.shape, f"Box shape: {path}")
                    require(
                        np.isfinite(pos).all() and np.isfinite(dim).all() and (dim > 0).all(), f"Box values: {path}"
                    )
                if "terrain_quat" in data:
                    require(
                        data["terrain_quat"].shape == (len(data["terrain_position"]), 4),
                        f"Box quaternion shape: {path}",
                    )
                    require(
                        np.allclose(np.linalg.norm(data["terrain_quat"], axis=-1), 1, atol=1e-4),
                        f"Box quaternion norm: {path}",
                    )
                if "terrain_vertices" in data:
                    vertices, faces = data["terrain_vertices"], data["terrain_faces"]
                    require(
                        vertices.ndim == 2 and vertices.shape[1] == 3 and np.isfinite(vertices).all(),
                        f"Mesh vertices: {path}",
                    )
                    require(
                        faces.ndim == 2 and faces.shape[1] == 3 and np.issubdtype(faces.dtype, np.integer),
                        f"Mesh faces: {path}",
                    )
                    require(faces.min() >= 0 and faces.max() < len(vertices), f"Mesh indices: {path}")
            probe = subprocess.check_output(
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
                    str(ROOT / variant["video"]),
                ],
                text=True,
            )
            stream = json.loads(probe)["streams"][0]
            a, b = map(float, stream["r_frame_rate"].split("/"))
            require(int(stream["nb_frames"]) == n and abs(a / b - variant["fps"]) < 1e-6, f"Video timing: {path}")
            pairs += 1
        for report in clip["reports"].values():
            require((ROOT / report).is_file(), f"Missing report: {report}")
    print(
        f"PASS: {len(catalog['clips'])} clips, {pairs} NPZ/video pairs. Geometry values and timing checked; dynamics not tested."
    )


if __name__ == "__main__":
    main()
