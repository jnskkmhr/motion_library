# Mixamo navigation motions for G1

This collection targets the project's risk-conditioned whole-body navigation:
stairs, ladders, ledges and slopes, vaults, crawling, crouching, jumps, sitting,
and recovery. It excludes dance, weapon animations, unrelated gestures, rope-only
climbing, and static poses. The earlier dance preview has been removed.

`navigation_manifest.json` now contains **seven reviewed candidates** from
[Linzhan Mou's Mixamo / UniMate release](https://huggingface.co/datasets/Linzhan/Mixamo-Animations-Characters):
a partial box climb, three crouch/duck motions, and three running gap jumps.
The manifest preserves source revision, filename, motion ID, and description.

The earlier 82-clip collection was too permissive. **75 pairings were removed from
the active data and public gallery**, including “Jump Up On Box.” Its platform
collapsed to ground height. Other exclusions include distorted ladders, unsupported
climbs/crawls, near-flat stairs, floating sitting poses, and ground-only motions.
[`rejected_clips.json`](rejected_clips.json) records the source and measured reason
for each exclusion. These seven retained clips are kinematic candidates, not ACP's
exact data or demonstrations validated by a tracking policy.

Each clip retains source timing and is capped at **60 seconds** before retargeting.
Short source clips are not stretched or looped to fill a minute. The source index reports keyframe counts, not duration; the importer samples the
actual FBX action timeline.

## Generate the collection

The base environment uses Python 3.12. Blender's `bpy` importer uses a separate,
uv-managed Python 3.11 environment:

```bash
uv sync --locked
UV_PROJECT_ENVIRONMENT=.venv-mixamo uv sync --python 3.11 --locked --extra mixamo
uv run python data_generator/mixamo/build_navigation.py \
  --isaaclab_python /path/to/IsaacLab/.venv/bin/python \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml
uv run python tools/build_catalog.py
uv run python tools/build_catalog.py --include-local
uv run python tools/validate_library.py
uv run python tools/validate_library.py --catalog data/catalog.local.json
```

`--limit 1` runs a small initial build. `--resume` authorizes replacing an existing
final pair after the new build passes checks. Every build uses a temporary folder
under `outputs/` for raw FBX conversion, qpos, and USD export; it cleans up that
folder on success or failure. Only final paired files and reports enter `data/`.
`--workers` controls concurrent jobs (default 2). Individual failures are reported
in `outputs/mixamo_navigation_summary.json`, with logs in
`outputs/mixamo_navigation_logs/`. A failed batch returns a nonzero exit status.

The pipeline downloads only selected FBXs, runs G1 retargeting, exports body states
using the exact Isaac Lab ball-hand USD asset, fits static terrain, checks robot–terrain
penetration at every saved frame, screens joint limits and foot geometry, and
records a full 50 Hz video from the paired NPZ. Joint interpolation uses PCHIP to avoid
overshooting source joint limits. The recording frame count must match its NPZ.

## Inspect the results

- Public website: select **Mixamo**, then choose a **Motion type**.
- NPZ downloads: available directly from the public gallery and GitHub.
- Published data: `data/mixamo/<name>/motion.npz`, `motion.mp4`, and provenance/reports.
- Public files: `docs/previews/mixamo_<name>/` contains the rendered video,
  provenance, terrain fitting/collision reports, geometric report, and gallery metadata.

The public website contains final G1 NPZs with terrain, their matching videos,
and provenance/review reports. Source FBXs are not bundled.
The [Adobe terms](https://helpx.adobe.com/creative-cloud/faq/mixamo-faq.html) remain
applicable; a public mirror does not grant new redistribution or training rights.

## What retargeting does and does not solve

The importer reads the animated skeleton in meters/Z-up, estimates a scale from leg
lengths, and solves joint-limited inverse kinematics for G1's pelvis, torso, knees,
and ankles. Separate arm fitting matches upper-arm and forearm directions using
the actual G1 link geometry. Wrists remain neutral. Initial root XY is shifted to
zero; source motion timing and root displacement are retained after scaling.

## Paired terrain: the existing motion-library convention

Every final `motion.npz` includes `terrain_position` (box centers),
`terrain_dimension` (full box sizes), `terrain_quat` (xyzw), `terrain_id`, and
`ground_height`. Positions are in meters in the same world frame as the saved G1
body states. The existing Isaac Lab player spawns these boxes directly. There is
no separate Mixamo terrain format, and no motion arrays or timestamps are changed.

The source FBX does not supply obstacle assets. `fit_terrain.py` uses the shared
terrain fitters and the actual G1 USD geometry bounds. The retained scene types
are a climb box, low passage, and separated takeoff/landing platforms. Other
fitter modes remain available for experiments but are not accepted into this
reviewed subset.

`quality.py` rejects missing obstacles, missing jump gaps, endpoint foot-to-terrain
gaps above 8 cm (median over 0.1 s), and foot gaps above 8 cm during retained
non-jumping motions. The climb support estimate must be within 5 cm of its box
top. These are coarse exclusion thresholds, not claims of valid supporting contact.
Rejected source clips require a new manual motion/terrain review before restoration.

`terrain_fit.json` records dimensions, fitting decisions, estimated contact gaps,
and the final NPZ hash. `terrain_validation.json` checks all saved poses using the
ball-hand G1 MJCF collision shapes against the plane and every terrain box; the
build rejects penetration greater than 2 mm. It also compares common named body
positions against the USD-exported reference. The build stages new files and
promotes the NPZ and its video only after these checks and video timing checks pass.

These are kinematic candidates. Collision clearance does not establish supporting
contact: feet, hands, or pelvis can remain above the fitted surface. Sliding,
self-collision, balance, force feasibility, and dynamic tracking still need
validation before selecting clips for tracker training.

## Import another FBX

```bash
UV_PROJECT_ENVIRONMENT=.venv-mixamo uv run --python 3.11 --extra mixamo \
  python data_generator/mixamo/import_mixamo.py \
  --input /path/to/ClimbingLadder.fbx \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml \
  --output outputs/mixamo/ladder --max_duration 60
```

The importer requires Hips, Spine2, and standard Left/Right UpLeg, Leg, Foot, Arm,
ForeArm, and Hand bones; `mixamorig:` prefixes are accepted. `--animation_index`
selects the action. Optional `--terrain terrain.json` accepts canonical box/mesh
fields in the target G1 reference frame, after scaling and initial root XY shift.
It does not infer terrain from the animation. See the shared exporter and player
commands in `integrations/isaaclab/README.md` for individual outputs.
