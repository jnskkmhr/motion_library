# Extraction validation — 2026-10-23

- `uv sync --locked` succeeded in a fresh local environment; `uv lock --check` passed.
- The catalog contains 54 clips and 80 paired video/NPZ variants.
- `tools/validate_library.py` passed: file hashes, named array shapes, finite values,
  unit quaternions, terrain metadata, video frame counts, and video FPS.
- The extracted CLAW importer reproduced the saved hand-crawl qpos and labels exactly.
- Terrain normalization preserved joint and body-state arrays.
- Kimodo pipeline dry-run resolved the relocated generator, exporter, and player paths.
  Model generation was not rerun; copied outputs retain their original provenance.
- The extracted USD exporter produced a six-frame ball-hand G1 dataset with
  29 joints and 35 body frames in the existing Isaac Lab environment.
- The extracted Isaac Lab player recorded the missing platform before-refinement
  preview: 199 frames at 50 FPS. No motion data was regenerated for this preview.
- Browser checks passed for video playback, source/search filters, before/after
  switching, GitHub links, repeat control, empty results, mobile layout, and direct
  opening of docs/index.html. Desktop and mobile layouts were visually inspected.
- Scoped Ruff checks and Python compilation passed.

These checks establish packaging and playback consistency. They do not establish
contact feasibility, policy tracking success, or readiness of every clip for training.

# ACP source additions — 2026-10-23

- Added 22 OmniRetarget references: all 29 released climbing clips are now present.
  Each new clip was exported on the ball-hand G1 USD asset at 50 Hz and recorded
  in Isaac Lab with its original supplied terrain.
- The pinned OmniRetarget downloader/importer reproduced the saved climb_00 qpos,
  joint labels, and terrain exactly using a fresh download directory.
- Added five local LAFAN previews from the pinned public Unitree G1 retargeting:
  two walking windows, two falling/getting-up windows, and one jumping window.
  The second get-up window was trimmed to end after recovery, before the next fall.
  Source frame ranges and the root XY translation are recorded in provenance.
- All 27 new references passed the sampled joint-limit check. Foot-geometry reports
  are included; this is not a full contact/dynamic validation.
- At that stage, the public catalog contained 76 clips / 102 NPZ-video pairs,
  and the local catalog 81 clips / 107 pairs. See the navigation update below
  for current counts.
- Public and local galleries passed browser checks, including 29 OmniRetarget and
  five LAFAN selections, video loading, source/download links, and mobile layout.
- Mixamo FBX import uses the uv-locked Blender 4.5 Python 3.11 wheel. A synthetic
  animated skeleton passed the FBX transform/timing round trip and joint-limited
  G1 IK. The initial unrelated dance preview has been removed from the galleries.
- Python compilation, scoped Ruff checks, JavaScript syntax, and uv lock checks passed.

LAFAN derivatives, local catalogs, and Mixamo asset directories are ignored by Git.
No claim is made to reproduce ACP's exact demonstrations, retargeting, contact
labels, policy rollouts, or training results.

# Navigation collection — 2026-10-25

- Selected and downloaded 82 Mixamo motions from a pinned UniMate dataset revision:
  stairs, ladders, climbing/slopes, vaults, crawling/hand support, crouching/ducking,
  jumps/landings, sitting/standing, and recovery/rolls. No conversion jobs failed.
- All were retargeted to the 29-joint ball-hand G1 and exported with its exact
  Isaac Lab USD asset. Recordings have 50 Hz timing and match their saved NPZs.
- Each joint/root trajectory is distinct. All 82 sampled joint-position-limit
  checks pass. These checks do not validate speed limits, self-collisions, terrain
  contacts, balance, or dynamic tracking. Ground-contact errors remain.
- New clips total 221.22 seconds. Individual durations are 0.42–11.42 seconds,
  below the 60-second cap. No short clips were slowed down or repeated.
- A real FBX cap check requested one second and produced a 0.98-second 50 Hz
  export, with truncation recorded in provenance.
- Public catalog: 158 clips, 102 NPZ/video pairs, 82 video-only previews.
  Local catalog: 163 clips, 189 NPZ/video pairs. Full array, quaternion, geometry,
  checksum, video frame-count, and FPS validation passed for both catalogs.
- Sampled frames were inspected across the collection, with source-human/G1 crawl
  comparison. The FBX source does not supply obstacle assets; affected previews
  explicitly say “Obstacle not supplied.” These remain motion candidates for
  contact/terrain fitting and tracker evaluation, not validated demonstrations.
- Browser checks passed for all nine category counts and sample videos, direct
  clip links, search, mobile layout, retained OmniRetarget links, and local Mixamo
  NPZ downloads. Public previews do not advertise unavailable NPZ downloads.
- Scoped Ruff formatting/lint checks, JavaScript syntax and diff checks passed.
