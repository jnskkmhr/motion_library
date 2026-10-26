# Mixamo navigation motions for G1

This collection targets the project's risk-conditioned whole-body navigation:
stairs, ladders, ledges and slopes, vaults, crawling, crouching, jumps, sitting,
and recovery. It excludes dance, weapon animations, unrelated gestures, rope-only
climbing, and static poses. The earlier dance preview has been removed.

`navigation_manifest.json` selects 82 clips by their source descriptions from
[Linzhan Mou's Mixamo / UniMate release](https://huggingface.co/datasets/Linzhan/Mixamo-Animations-Characters).
The manifest pins the source revision and preserves each filename, motion ID,
description, and category. ACP cites Mixamo for stairs, ladders, and sitting;
these are relevant source candidates, not a reconstruction of ACP's exact data.

| Motion type | Selected clips |
| --- | ---: |
| Stairs | 4 |
| Ladder | 5 |
| Climb and slope | 10 |
| Vault | 3 |
| Crawl and hand support | 7 |
| Crouch and duck | 14 |
| Jump and landing | 21 |
| Sit and stand | 5 |
| Recovery and roll | 13 |

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

`--limit 6` runs a small initial batch. `--resume` retains outputs only when the
source request, retargeter, duration cap and robot MJCF match. Terrain fitting is
repeated when the raw motion, selected terrain kind, or fitting code changes; its
previous video is then invalidated. `--workers` controls
concurrent conversion/render processes (default 2). Individual failures are
reported in `outputs/mixamo_navigation_summary.json`; per-stage logs are in
`outputs/mixamo_navigation_logs/`. A failed batch returns a nonzero exit status.

The pipeline downloads only selected FBXs, runs G1 retargeting, exports body states
using the exact Isaac Lab ball-hand USD asset, fits static terrain, checks robot–terrain
penetration at every saved frame, screens joint limits and foot geometry, and
records a full 50 Hz video from the paired NPZ. Joint interpolation uses PCHIP to avoid
overshooting source joint limits. The recording frame count must match its NPZ.

## Inspect the results

- Public website: select **Mixamo**, then choose a **Motion type**.
- Local website: open `docs/local.html` for converted NPZ download links.
- Local data: `data/mixamo/<name>/motion.npz`, `motion.mp4`, and provenance/reports.
- Public files: `docs/previews/mixamo_<name>/` contains the rendered video,
  provenance, terrain fitting/collision reports, geometric report, and gallery metadata.

The public website contains videos, not source FBXs or converted animation arrays.
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

The source FBX does not supply obstacle assets. `fit_terrain.py` follows the shared
CLAW/terrain fitters: it transforms the actual G1 USD geometry bounds through the
reference motion and fits static boxes around the swept robot geometry. Each
manifest entry explicitly selects its terrain type:

- Stairs use low-speed foot intervals and foot-edge clearance.
- Ramps use a tilted box beneath the stance-foot samples.
- Climbs, vaults, and sitting use support boxes near elevated feet, hands, or pelvis.
- Ladders use rungs and connected side rails. Rung offsets avoid the swept robot;
  these can form a nonuniform climbing frame rather than a regular straight ladder.
- Crawls and ducking use a low roof and posts, following the CLAW overhead example.
- Traveling jumps use takeoff/landing platforms when an airborne interval permits
  a gap. In-place jumps use one platform. The floor below a gap is lowered.
- Ground transitions and recovery use a bounded ground slab.

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
