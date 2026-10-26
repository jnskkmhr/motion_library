"""Screen the reviewed Mixamo obstacle pairings before publishing their final files."""

import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation

from tools.terrain_contract import require_scene


def review(motion, fit_path, output):
    """Reject missing obstacles and large foot-to-terrain gaps; this is not dynamics."""
    with np.load(motion, allow_pickle=False) as archive:
        data = dict(archive)
    require_scene(data, motion)
    fit = json.loads(fit_path.read_text())
    corners = data["foot_box_corners_w"]
    distances = []
    for center, size, quat in zip(
        data["terrain_position"], data["terrain_dimension"], data["terrain_quat"], strict=True
    ):
        local = Rotation.from_quat(quat).inv().apply((corners - center).reshape(-1, 3)).reshape(corners.shape)
        outside = np.abs(local) - size / 2
        distances.append(np.linalg.norm(np.maximum(outside, 0), axis=-1) + np.minimum(outside.max(axis=-1), 0))
    foot_gap = np.min(distances, axis=0).min(axis=(1, 2))
    window = max(1, round(0.1 * float(data["fps"])))
    start, end = float(np.median(foot_gap[:window])), float(np.median(foot_gap[-window:]))
    kind = fit["terrain_kind"]
    reasons = []
    if kind not in ("gap", "passage", "climb"):
        reasons.append("Terrain/action type needs a new manual review before active-library use")
    if max(start, end) > 0.08:
        reasons.append("Nearest foot exceeds 0.08 m terrain gap at a clip endpoint")
    if kind != "gap" and foot_gap.max() > 0.08:
        reasons.append("Both feet exceed 0.08 m terrain gap during a non-jumping motion")
    if kind == "gap" and fit.get("gap_width_m", 0) <= 0.15:
        reasons.append("No traversed gap greater than 0.15 m")
    if kind == "climb" and fit.get("support_point_to_box_top_gap_m", float("inf")) > 0.05:
        reasons.append("Estimated support point exceeds 0.05 m gap to its box top")
    roles = set(fit["terrain_roles"]) - {"ground_slab"}
    if not roles:
        reasons.append("Ground slab alone does not supply an obstacle")
    report = {
        "status": "rejected" if reasons else "retained_kinematic_candidate",
        "motion_sha256": hashlib.sha256(motion.read_bytes()).hexdigest(),
        "nearest_foot_endpoint_gap_m": [start, end],
        "maximum_nearest_foot_gap_m": float(foot_gap.max()),
        "terrain_kind": kind,
        "rejection_reasons": reasons,
        "limitations": "Coarse kinematic screening, not validated support contact, balance, self-collision or tracking.",
    }
    output.write_text(json.dumps(report, indent=2) + "\n")
    if reasons:
        raise ValueError(f"Rejected motion/terrain pairing {motion}: {'; '.join(reasons)}")
    return report
