# LAFAN: local G1 previews

ACP cites LAFAN for walking and getting up. We use the publicly released
[Unitree G1 retargeting](https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset),
pinned at `ce1572906efe6157840e8474d5a0d7aa87481e74`. This is not ACP's own retargeting
or a claim to reproduce its exact clip selection.

The upstream CSV has root XYZ + quaternion xyzw + 29 joint angles at 30 Hz.
`import_lafan.py` names those joints, selects an explicit time window, and translates
the first root XY to zero. It preserves root Z, heading, joint angles, and timing.
Final USD export resamples to 50 Hz on our ball-hand G1. Flat ground is explicitly
Z=0; these CSVs do not supply obstacle geometry.

```bash
uv run python data_generator/lafan/import_lafan.py \
  --clip fallAndGetUp1_subject1.csv --start 2 --end 12 \
  --output outputs/lafan/get_up
```

The source is downloaded automatically at the pinned revision. `--source` can
instead point to an already downloaded CSV. Outputs are `g1_qpos.npz` and
`provenance.json`; use the shared exporter for final body-state NPZs.

For all five selected examples, including videos:

```bash
uv run python data_generator/build_acp_library.py --sources lafan \
  --isaaclab_python /path/to/IsaacLab/.venv/bin/python \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml
uv run python tools/build_catalog.py --include-local
uv run python tools/validate_library.py --catalog data/catalog.local.json
uv run python -m http.server 8000 --bind 127.0.0.1
```

Open <http://localhost:8000/docs/local.html>. The selection windows are recorded
in `data_generator/acp_sources.json`, with inclusive endpoints on 30 Hz timestamps.
Use `--resume` only for an unchanged interrupted request.

[LAFAN's source terms](https://github.com/ubisoft/ubisoft-laforge-animation-dataset#readme)
are CC BY-NC-ND 4.0. The upstream G1 dataset card also points to those terms.
Converted NPZs/videos stay under ignored `data/lafan/<clip>/` directories. The
public repository provides scripts and instructions, not those derived files.
The local gallery uses upstream source links rather than nonexistent GitHub NPZ links.
