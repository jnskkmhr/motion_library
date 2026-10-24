# Kimodo generation

This directory contains our Kimodo integration, not a copy of the upstream model.
The root `pyproject.toml` pins [Kimodo](https://github.com/nv-tlabs/kimodo) to commit
`e1c219c99b4b94ba48be1fb6a1cbc64ecee4c954`; `uv.lock` records its dependencies.

```bash
uv sync --locked --extra kimodo
uv run hf auth login
uv run python data_generator/kimodo/generate.py \
  --prompt "A person walks forward." --duration 5 --seed 44 \
  --output outputs/kimodo/walk_seed44
```

This uses `Kimodo-G1-RP-v1` directly, not SOMA/GMR retargeting. Generation needs
access to Kimodo's upstream model dependencies, including its gated Llama text
encoder. The original run used an approved Hugging Face account. No credentials
or weights are stored here.

`generate.py` writes raw Kimodo output, G1 qpos, and provenance. For a supplied
Kimodo G1 example, use `--example /path/to/motion.npz` instead of `--prompt`.
This imports a sample; it does not generate a new motion.

## Export and record

Set `ISAACLAB_PYTHON` to an existing Isaac Lab interpreter with the ball-hand G1
asset configuration. This is separate from the uv environment containing Kimodo.

```bash
ISAACLAB_PYTHON=/path/to/IsaacLab/.venv/bin/python
uv run python data_generator/kimodo/run_pipeline.py \
  --kimodo_python "$PWD/.venv/bin/python" --isaaclab_python "$ISAACLAB_PYTHON" \
  --prompt "A person walks forward." --duration 5 --seed 44 \
  --output outputs/kimodo/walk_seed44 --video --dry_run
```

Remove `--dry_run` to execute. Use a new output directory for each run.
Optional `--terrain stairs` fits ascending stair boxes; `--terrain overhead`
fits a low passage. Neither is a general motion/terrain optimizer.

For terrain authored before generation, use `--platform --robot_mjcf /path/to/g1.xml`
instead of `--prompt`. `--platform_height 0.15` creates a 15 cm high platform.
The sparse constraints and contact refinement use that same fixed geometry.

## Batch candidates

```bash
uv run python data_generator/kimodo/generate_batch.py --output outputs/kimodo/raw --dry_run
uv run python data_generator/kimodo/run_terrain_batch.py \
  --kimodo_python "$PWD/.venv/bin/python" --isaaclab_python "$ISAACLAB_PYTHON" \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml \
  --output outputs/kimodo/terrain_batch --workers 2 --video
```

The default manifest requests crouch, crawl, ramp, bear walk, and gap-jump motions.
`--manifest data_generator/kimodo/long_jump_batch.json` requests 1.4 m gap examples.
`--process_existing` resumes the same saved requests, not a changed configuration.

`motion_generated` is the generated motion with authored terrain. `motion` applies
contact refinement. Both are G1 exports, not raw human mocap. Validation measures
sampled geometry and joint limits; it does not prove dynamic trackability.

Existing candidates are in [`data/kimodo`](../../data/kimodo). They remain
**experimental**: the user judged their naturalness worse than the public mocap
references. Keep failed candidates labeled and separate from training selections.
The supplied stair sample is under `supplied_stairs`; it was imported, not generated.
