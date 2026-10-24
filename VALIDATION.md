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
