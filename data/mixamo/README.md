# Mixamo navigation motions

Seven reviewed box-climb, crouch/duck, and running-gap-jump candidates remain.
The earlier 75 poor pairings are excluded; see
[`rejected_clips.json`](../../data_generator/mixamo/rejected_clips.json).
The active source selection is
[`navigation_manifest.json`](../../data_generator/mixamo/navigation_manifest.json).

Each published clip has `data/mixamo/<name>/motion.npz`, its matching video,
and provenance/review reports. The public gallery provides GitHub and direct
NPZ download links. Raw source FBXs and rejected candidates are not bundled.

Every final NPZ includes fitted terrain using the existing `terrain_position`,
`terrain_dimension` (full sizes), `terrain_quat` (xyzw), and `ground_height` fields.
The accompanying video renders those same boxes. Fitting and collision reports
are saved beside the NPZ. These are kinematic candidates: fitted geometry is not
source Mixamo geometry, and support contacts and dynamic tracking remain unvalidated.
See the [generator README](../../data_generator/mixamo/README.md).
