# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Generate diverse G1 candidates, loading Kimodo and its text encoder only once.

Outputs are raw candidates, not accepted terrain pairs or dynamically validated
motions. Use export_mimic.py for exact-asset FK and review each contact sequence
before constructing terrain. In particular, fit_stairs.py does not support ramps,
crawling, or vaulting.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("diverse_motions.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--only", nargs="+", help="Select motion names from the manifest.")
    parser.add_argument(
        "--dry_run", action="store_true", help="Print requests without loading models or writing outputs."
    )
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    seeds, motions = manifest["seeds"], manifest["motions"]
    names = [motion["name"] for motion in motions]
    if not seeds or any(type(seed) is not int or seed < 0 for seed in seeds) or len(set(seeds)) != len(seeds):
        parser.error("Seeds must be distinct nonnegative integers.")
    if (
        not names
        or len(set(names)) != len(names)
        or any(not name or not all(c in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in name) for name in names)
    ):
        parser.error("Motion names must be distinct lowercase letters, digits, and underscores.")
    if args.only and set(args.only) - set(names):
        parser.error(f"Unknown motion names: {sorted(set(args.only) - set(names))}")
    requests = []
    for motion in motions:
        if args.only and motion["name"] not in args.only:
            continue
        if not motion["prompt"].strip() or not math.isfinite(motion["duration"]) or motion["duration"] <= 0:
            parser.error("Each motion needs a nonempty prompt and a positive finite duration.")
        for seed in seeds:
            requests.append({**motion, "seed": seed, "directory": f"{motion['name']}/seed_{seed}"})
    print(f"{len(requests)} candidates; {sum(item['duration'] for item in requests):.1f} seconds of requested motion")
    for item in requests:
        print(f"  {item['directory']}: {item['duration']} s — {item['prompt']}")
    if args.dry_run:
        return
    for item in requests:
        destination = args.output / item["directory"]
        if destination.exists() and any(destination.iterdir()):
            raise FileExistsError(f"Refusing to overwrite {destination}; use a new output folder or --only.")

    import numpy as np
    from generate import save_motion
    from kimodo import load_model
    from kimodo.tools import seed_everything

    model_name = "Kimodo-G1-RP-v1"
    model = load_model(model_name, device="cuda")
    args.output.mkdir(parents=True, exist_ok=True)
    status = [{**item, "status": "pending"} for item in requests]
    status_file = args.output / "batch_status.json"
    status_file.write_text(json.dumps(status, indent=2) + "\n")
    for item in status:
        try:
            seed_everything(item["seed"])
            batch = model(
                [item["prompt"]],
                [round(item["duration"] * model.fps)],
                num_samples=1,
                num_denoising_steps=100,
                post_processing=False,
                return_numpy=True,
            )
            motion = {key: value[0] for key, value in batch.items() if isinstance(value, np.ndarray)}
            save_motion(
                motion,
                model.skeleton,
                model.fps,
                args.output / item["directory"],
                {
                    "model": model_name,
                    "source_type": "generated",
                    "prompt": item["prompt"],
                    "seed": item["seed"],
                    "duration": item["duration"],
                    "retargeting": "official Kimodo G1 joint projection; no SOMA/GMR",
                    "terrain_fitted": False,
                    "dynamic_tracking_validated": False,
                },
            )
            item["status"] = "generated_unvalidated"
        except Exception as error:
            item.update(status="failed", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            status_file.write_text(json.dumps(status, indent=2) + "\n")


if __name__ == "__main__":
    main()
