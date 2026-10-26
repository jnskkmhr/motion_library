# Local Mixamo navigation motions

Selected stair, ladder, slope, climb, vault, crawl, crouch, jump, sit/stand, and
recovery clips are retargeted to ball-hand G1. Each output is at most 60 seconds.
The exact selection and source revision are in
[`navigation_manifest.json`](../../data_generator/mixamo/navigation_manifest.json).

Public previews live under `docs/previews/mixamo_<name>/`. Converted NPZs remain
local under `data/mixamo/<name>/motion.npz`. Build `docs/local.html` with
`uv run python tools/build_catalog.py --include-local` for local NPZ downloads.

Every final NPZ includes fitted terrain using the existing `terrain_position`,
`terrain_dimension` (full sizes), `terrain_quat` (xyzw), and `ground_height` fields.
The accompanying video renders those same boxes. Fitting and collision reports
are saved beside the NPZ. These are kinematic candidates: fitted geometry is not
source Mixamo geometry, and support contacts and dynamic tracking remain unvalidated.
See the [generator README](../../data_generator/mixamo/README.md).
