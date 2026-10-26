# Local Mixamo navigation motions

Seven reviewed box-climb, crouch/duck, and running-gap-jump candidates remain.
The earlier 75 poor pairings are excluded; see
[`rejected_clips.json`](../../data_generator/mixamo/rejected_clips.json).
The active source selection is
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
