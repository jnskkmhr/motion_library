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
- Public catalog: 77 clips / 102 NPZ-video pairs plus one video-only preview.
  Local catalog: 82 clips / 108 NPZ-video pairs.
  Array/terrain integrity, hashes, video frame counts and FPS passed for all pairs.
- Public and local galleries passed browser checks, including 29 OmniRetarget and
  five LAFAN selections, video loading, source/download links, and mobile layout.
- Mixamo FBX import uses the uv-locked Blender 4.5 Python 3.11 wheel. A synthetic
  animated skeleton passed the FBX transform/timing round trip and joint-limited
  G1 IK. The real Three.js r180 Mixamo Samba sample also passes import and exact-asset
  export: 911 frames at 50 Hz. The paired G1 video was visually inspected.
  Joint limits pass, but foot sliding and up to 6.9 cm of ground penetration
  remain; it is explicitly labeled an unvalidated importer demonstration.
- Browser checks cover the Mixamo tab, video playback, direct clip link, source
  attribution, hidden public NPZ download, restored ordinary NPZ links, local
  NPZ access, and mobile layout.
- Python compilation, scoped Ruff checks, JavaScript syntax, and uv lock checks passed.

LAFAN derivatives, local catalogs, and Mixamo asset directories are ignored by Git.
No claim is made to reproduce ACP's exact demonstrations, retargeting, contact
labels, policy rollouts, or training results.
