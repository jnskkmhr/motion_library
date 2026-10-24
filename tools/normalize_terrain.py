"""Write a canonical terrain NPZ without changing motion arrays."""

import argparse
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ground_height", type=float, help="Explicit source-scene ground Z in meters, if needed.")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; choose a new file.")
    with np.load(args.input, allow_pickle=False) as source:
        data = dict(source)
    if "terrain_boxes" in data:
        if "terrain_position" in data or "terrain_dimension" in data:
            parser.error("Input contains both terrain schemas.")
        boxes = data.pop("terrain_boxes")
        data["terrain_position"], data["terrain_dimension"] = boxes[:, :3], boxes[:, 3:]
    if "terrain_box_quat" in data:
        if "terrain_quat" in data:
            parser.error("Input contains both terrain quaternion schemas.")
        data["terrain_quat"] = data.pop("terrain_box_quat")
    if args.ground_height is not None:
        if not np.isfinite(args.ground_height):
            parser.error("Ground height must be finite.")
        if "ground_height" in data and float(data["ground_height"]) != args.ground_height:
            parser.error("Refusing to replace a different source ground height.")
        data["ground_height"] = np.asarray(args.ground_height)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        np.savez_compressed(stream, **data)


if __name__ == "__main__":
    main()
