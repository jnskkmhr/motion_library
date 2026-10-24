# Mimic development handoff

Updated: 2026-10-21. This is project context for the next agent, not a record of a completed training run.

## Active checkout and user decisions

Continue development in `/home/jkamohara/isaac/tmp/isaac_PR/IsaacLab`, branch
`dokyun/multi-mimic` (branch verified when this file was written). The user explicitly
selected this checkout for future mimic work. Check the current branch and diff
before editing; it contains unrelated work and uncommitted terrain integration.
Do not reset, overwrite, commit, or push that work without the appropriate request.
Follow this checkout's `AGENTS.md` and contribution guide.

The original generation workspace is `/home/jkamohara/isaac/skill_chaining/IsaacLab`.
Offline generation and retargeting remain under its
`source/isaaclab_tasks/isaaclab_tasks/contrib/multi_task_locomotion/` directory.
The user requested only runtime motion loading and terrain integration inside mimic,
with utilities beside `mdp/`, not inside `data/motions/`.

## Research direction

The intended task is risk-conditioned whole-body navigation from A to B. Candidate
motions include crouching, crawling, ramp climbing, walking with hand support,
large-gap jumps, and climbing/vaulting with hand contact. The user's requested jump
gap was approximately 1–1.75 m. Longer-term contributions are agent-assisted skill
trajectory synthesis across terrain, risk conditioning to select different behavior,
and demonstration on a full-scale humanoid.

A motion reference is a temporal trajectory paired with terrain, not just a skill
label. Earlier GNN/graph-planner designs were exploratory; the discussion later
returned to agent-assisted reference synthesis and closed-loop teacher–student
training. Do not treat a GNN, Transformer, distributional PPO, or a particular
risk objective as an implemented or settled design. Current code supports motion
tracking with paired terrain, not the complete navigation method.

The user prefers plain research language, explained terms, concise diagrams, and
explicit coordinate/contact definitions. Their requested robot variant has fixed
spherical hand pads while retaining 29 actuated joints. The hand pads are simulation
variants; Isaac Lab should continue using USD assets. Original asset sources were
also requested under `/home/jkamohara/isaac/newton-assets`; verify that repository's
current state before asserting which variants are available there.

## Installed reference data

Relative to `source/isaaclab_tasks/isaaclab_tasks/contrib/mimic/`:

- `data/motions/data/parc/jump_001` through `jump_005`: 5 corrected parkour jumps.
- `data/motions/data/deep_wholebody_parkour/`: dive_roll, jump_sit, kneel_climb, roll_vault.
- `data/motions/data/omniretarget/`: climb_06, 08, 14, 16, 20, 26, 28.
- `data/motions/data/claw/`: 00452_crouch, 00543_hand_crawl, 02067_elbow_crawl.

Each directory has `motion.npz`, `provenance.json`, and `source_validation.json`.
See `data/motions/data/GENERATED_MOTIONS.md` and `generated_motion_manifest.json`
for the inventory and provenance. There are 19 installed clips, 7,796 frames in
all, at 50 Hz, with 29 joints and 35 saved bodies. PARC uses the corrected
`parc_anatomical_arms_v2` data. Kimodo clips were explicitly excluded by the user
because of their motion quality. Do not reintroduce them as training references.

Installation preserved the source motion arrays. Terrain metadata was normalized
to the schema below. Explicit `ground_height=0` was added to CLAW/OmniRetarget
copies to preserve their source viewer's ground surface. Existing negative ground
heights in the other sources were retained. References were retargeted for the
ball-hand G1; successful kinematic playback does not establish dynamic feasibility.

## Runtime code and terrain contract

- `mdp/motion_loader.py`: existing motion-state loader.
- `mdp/commands.py`: selects clips/reference times and applies their terrain origins.
- `utils/motion_paired_terrain_generator.py`: terrain metadata, validation, geometry deduplication/layout.
- `utils/motion_paired_terrain_cfg.py`: configuration helpers.
- `utils/motion_paired_terrain_importer.py`: shared static terrain construction.
- `utils/motion_env.py`: `MotionTrackingEnv` selects terrain after CLI overrides,
  before scene creation.
- `utils/README.md`: maintained schema and usage documentation.
- `config/g1_29dof/env_cfg/commands_cfg.py`: rigid G1 motion configuration.
- `data/motions/play_motion_file.py`: standalone kinematic reference player.

Terrain and body states use the same reference coordinate frame, in meters:

| NPZ key | Shape | Meaning |
| --- | --- | --- |
| `terrain_position` | `(N, 3)` | Box center positions |
| `terrain_dimension` | `(N, 3)` | Full box sizes along each box's local axes |
| `terrain_quat` | `(N, 4)` | Box orientations, xyzw; optional identity |
| `terrain_vertices` | `(V, 3)` | Triangle mesh vertices |
| `terrain_faces` | `(F, 3)` | Triangle indices |
| `ground_height` | scalar | Optional ground top Z in the reference frame |
| `terrain_id` | scalar string | Optional geometry identifier |

Use box position/dimension and mesh vertex/face pairs together. Legacy
`terrain_boxes` and `terrain_box_quat` are read for compatibility but new data must
use the separate keys. Terrain quaternion order is always xyzw, independently of
motion quaternion metadata.

Distinct terrain geometries occupy separate shared static patches. At reset the
motion command selects a clip and assigns its patch translation to that robot
environment's origin. Robot/reference positions receive the same translation as
the terrain. Identical geometry can share a patch. This is not terrain respawning
for each robot reset. Ground slabs are bounded to patches; no implicit plane at
Z=0 fills gaps. Paired terrain disables independent reference XY/yaw offsets.

`terrain_mode` is `auto`, `motion`, or `scene`:

- `auto`: all clips have terrain → use it; none have terrain → scene terrain.
  A mixed collection raises an error. Flat clips can declare `ground_height`.
- `motion`: every clip must have terrain metadata.
- `scene`: ignore NPZ terrain and retain the task's scene terrain.

Both rigid G1 task registrations use `MotionTrackingEnv`. Soft-terrain variants
retain their own setup. Rewards and observations were not redesigned for these
new contact-rich motions.

## Commands

Run from the active checkout. The default LAFAN references are not bundled, so
supply an installed motion explicitly. Quote paths inside the CLI list as below.

```bash
cd /home/jkamohara/isaac/tmp/isaac_PR/IsaacLab
MIMIC_MOTION="$PWD/source/isaaclab_tasks/isaaclab_tasks/contrib/mimic/data/motions/data/parc/jump_005/motion.npz"

uv run python source/isaaclab_tasks/isaaclab_tasks/contrib/mimic/data/motions/play_motion_file.py \
  --motion_file "$MIMIC_MOTION" --terrain_mode motion

uv run python source/isaaclab_tasks/isaaclab_tasks/contrib/mimic/data/motions/play_motion_file.py \
  --motion_file "$MIMIC_MOTION" --terrain_mode motion --headless --video

uv run isaaclab train --rl_library rsl_rl --task IsaacContrib-Mimic-G1-29dof-v0 \
  --num_envs 1024 --logger tensorboard --viz none \
  "env.commands.motion.motion_files=['$MIMIC_MOTION']" \
  env.commands.motion.terrain_mode=motion env.sim.dt=0.005

uv run isaaclab play --rl_library rsl_rl --task IsaacContrib-Mimic-G1-29dof-Play-v0 \
  --checkpoint /absolute/path/to/model_500.pt --num_envs 1 --viz newton_gl \
  "env.commands.motion.motion_files=['$MIMIC_MOTION']" \
  env.commands.motion.terrain_mode=motion env.sim.dt=0.005
```

`uv run python` launches the standalone reference script; `uv run isaaclab play`
loads a trained policy checkpoint. These are different operations. The 50 Hz data
requires `sim.dt=0.005` with decimation 4; the default task control rate is 60 Hz.
The runtime does not resample motion. Recheck decimation if task settings change.

## Playback fix and validation

The player previously advanced frames without wall-clock pacing while its window
presented at a default 30 FPS. It also wrapped finite clips to frame zero. The fix
paces interactive replay at the source FPS, configures that FPS on the window,
and initially held the last pose by default. The user subsequently requested
repeating playback: looping is now the default, and `--no-loop` holds the final
pose. Looping does not blend the end into the beginning. Headless video export remains unthrottled
and encodes at the reference FPS. Slow rendering can slow playback; reference
frames are not intentionally skipped to catch up.

Playback validation on PARC jump_005 checked final-pose holding and explicit loop
wrapping. A real interactive run recorded 100 frames at 50 Hz; elapsed time between
the first and last recorded frames was 1.980 seconds (expected 1.980 seconds).
Headless export also produced 8 frames at 50 FPS (0.160 seconds). Scoped
formatting/lint checks and the playback changelog fragment validation passed.

Earlier integration checks loaded all 19 clips with `MotionLibrary` and constructed
17 distinct terrain patches. A real four-environment run exercised three clips,
partial resets, 24 steps, and terrain contacts. Two focused terrain-selection tests
passed. Reference video export was also checked. These are loading/integration
checks, not evidence of a trained tracker or successful traversal.

## Outstanding work and cautions

1. Verify the robot asset used by training and replay against reference metadata.
   The standalone player currently uses `UNITREE_G1_29DOF_BOX_FOOT_CFG`; it does not
   automatically select the ball-hand variant from the NPZ `robot_asset` field.
   Joint count alone does not validate hand geometry, body frames, or contact alignment.
2. Inspect contact-rich clips against the actual robot collision shapes and terrain,
   then train a small tracking experiment. No successful full training run on this
   new collection has been established. Source validation files do not prove stability.
3. Distinguish source/retargeting artifacts from display timing. The playback fix
   does not alter trajectories or smooth any abrupt poses within a clip.
4. Preserve unrelated perception, PPO, ghost-robot, and OSMO edits in the checkout,
   as well as deliberate old-data deletions. Verify the diff before integration.
5. The checkout's repository-wide changelog check previously reported unrelated
   branch errors; scoped file checks passed. Diagnose current results rather than
   treating that prior status as a permanent exemption.

Temporary validation worktrees/logs under `/tmp` may disappear. The maintained
runtime documentation and installed dataset manifests are the durable references.
