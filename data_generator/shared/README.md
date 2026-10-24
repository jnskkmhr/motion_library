# Shared G1 export and terrain tools

`export_mimic.py` resamples root poses and joint angles, then computes body states
with the exact target USD asset in Isaac Lab. Joint interpolation uses PCHIP,
root translation uses cubic splines, and root orientation uses SLERP.

These tools need an existing Isaac Lab environment providing
`UNITREE_G1_29DOF_BALL_HAND_CFG`, `isaaclab_newton`, and the USD Python bindings.
The original generation checkout was `~/isaac/skill_chaining/IsaacLab`. Core data
inspection and source importers run in this repository's uv environment.

```bash
ISAACLAB_PYTHON=/path/to/IsaacLab/.venv/bin/python
uv run --no-project --python "$ISAACLAB_PYTHON" python data_generator/shared/export_mimic.py \
  --input outputs/claw/00543_hand_crawl/g1_qpos.npz \
  --output outputs/claw/00543_hand_crawl/motion_raw.npz \
  --robot_asset UNITREE_G1_29DOF_BALL_HAND_CFG

uv run --no-project --python "$ISAACLAB_PYTHON" python data_generator/shared/fit_overhead.py \
  --input outputs/claw/00543_hand_crawl/motion_raw.npz \
  --output outputs/claw/00543_hand_crawl/motion.npz
```

For supplied terrain, export directly to `motion.npz` and do not run a fitter.
`fit_stairs.py` fits ascending steps below foot-support intervals; `fit_overhead.py`
fits a roof/posts around forward crouching or crawling. The latter uses USD body
bounds. `build_ball_hand_asset.py` derives the USD hand-pad variant from matching
ball-hand URDF/MJCF assets. It does not bundle or download robot assets.

The preserved generation tools still use legacy `terrain_boxes` internally.
Before adding a new export to `data/`, run `tools/normalize_terrain.py` to create
the separate `terrain_position`, `terrain_dimension`, and `terrain_quat` keys.
The shared Isaac Lab player accepts both schemas. Body-state arrays are unchanged
by normalization; an optional explicit ground height must reflect the source scene.

See [Isaac Lab playback](../../integrations/isaaclab/README.md) and
[the dataset contract](../../data/README.md).
