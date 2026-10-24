"""Import a time window of Unitree's published LAFAN1 G1 retargeting."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from huggingface_hub import hf_hub_download

REPO = "lvhaidong/LAFAN1_Retargeting_Dataset"
REVISION = "ce1572906efe6157840e8474d5a0d7aa87481e74"
JOINT_NAMES = (
    [
        f"{s}_{j}_joint"
        for s in ("left", "right")
        for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
    ]
    + [f"waist_{j}_joint" for j in ("yaw", "roll", "pitch")]
    + [
        f"{s}_{j}_joint"
        for s in ("left", "right")
        for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
    ]
)


def import_clip(clip, start, end, output, source=None):
    if Path(clip).name != clip or not clip.endswith(".csv"):
        raise ValueError("Clip must be a CSV basename from the upstream g1 directory.")
    if not np.isfinite([start, end]).all() or start < 0 or end <= start:
        raise ValueError("Require finite 0 <= start < end, in seconds.")
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"Choose a new output directory: {output}")
    if source is None:
        source = Path(hf_hub_download(REPO, f"g1/{clip}", repo_type="dataset", revision=REVISION))
    qpos = np.loadtxt(source, delimiter=",")
    if qpos.ndim != 2 or qpos.shape[1] != 36 or not np.isfinite(qpos).all():
        raise ValueError("Expected finite root XYZ + quaternion xyzw + 29 joint columns.")
    if not np.allclose(np.linalg.norm(qpos[:, 3:7], axis=1), 1, atol=2e-4):
        raise ValueError("Source root quaternions are not normalized.")
    first, last = int(round(start * 30)), int(round(end * 30))
    if abs(first / 30 - start) > 1e-6 or abs(last / 30 - end) > 1e-6 or last >= len(qpos) or last - first < 2:
        raise ValueError("Window must contain >=3 source frames and fall on valid 30 Hz timestamps.")
    selected = qpos[first : last + 1].copy()
    # Translation only; preserve root Z, heading, joints, and every source timestamp.
    origin = selected[0, :2].copy()
    selected[:, :2] -= origin
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output / "g1_qpos.npz",
        qpos=selected,
        fps=30,
        joint_names=JOINT_NAMES,
        quaternion_order="xyzw",
        ground_height=0.0,
    )
    provenance = {
        "dataset": REPO,
        "revision": REVISION,
        "source_url": f"https://huggingface.co/datasets/{REPO}/blob/{REVISION}/g1/{clip}",
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_clip": clip,
        "source_fps": 30,
        "first_frame": first,
        "last_frame_inclusive": last,
        "source_window_s": [start, end],
        "root_xy_translation_m": (-origin).tolist(),
        "retargeting": "Upstream Unitree G1 retarget; no additional IK or joint editing",
        "terrain_source": "Explicit flat ground at reference Z=0; no obstacle geometry supplied",
        "license": "LAFAN1 CC BY-NC-ND 4.0; keep derived exports local",
        "public_distribution": False,
        "exact_acp_clip": False,
        "acp_relationship": "ACP cites LAFAN for walking/get-up; this is a different public G1 retargeting",
        "dynamic_tracking_validated": False,
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--clip", required=True)
    p.add_argument("--start", type=float, required=True)
    p.add_argument("--end", type=float, required=True)
    p.add_argument("--source", type=Path, help="Optional already-downloaded source CSV")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    import_clip(a.clip, a.start, a.end, a.output, a.source)


if __name__ == "__main__":
    main()
