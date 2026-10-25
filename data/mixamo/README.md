# Local Mixamo navigation motions

Selected stair, ladder, slope, climb, vault, crawl, crouch, jump, sit/stand, and
recovery clips are retargeted to ball-hand G1. Each output is at most 60 seconds.
The exact selection and source revision are in
[`navigation_manifest.json`](../../data_generator/mixamo/navigation_manifest.json).

Public previews live under `docs/previews/mixamo_<name>/`. Converted NPZs remain
local under `data/mixamo/<name>/motion.npz`. Build `docs/local.html` with
`uv run python tools/build_catalog.py --include-local` for local NPZ downloads.

These are kinematic candidates. Source obstacle geometry is unavailable; the
reference ground plane is not a fitted staircase, ladder, box, or chair.
See the [generator README](../../data_generator/mixamo/README.md).
