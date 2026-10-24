# OmniRetarget import

Imports already-retargeted G1 joint/root trajectories and their supplied terrain
from [OmniRetarget_Dataset](https://huggingface.co/datasets/omniretarget/OmniRetarget_Dataset).
The terrain is preserved rather than fitted. The source dataset card recorded MIT.

Download revision `135259130158f1cb1799e4ab9b6566c938285fc9` and extract its G1
robot–terrain data. The importer expects this layout under `--source`:

```text
models/g1/g1_29dof_spherehand.urdf
models/terrain/climb_14/multi_boxes_z_scale_1.0.urdf
robot-terrain/climb_14_z_scale_1.0.npz
```

Keep the terrain URDF's referenced mesh files alongside it.

```bash
uv run python data_generator/omniretarget/import_omniretarget.py \
  --source /path/to/extracted/OmniRetarget \
  --clips climb_06 climb_08 climb_14 climb_16 climb_20 climb_26 climb_28 \
  --output outputs/omniretarget
```

Then use the [shared USD exporter](../shared/README.md). The importer converts
Drake's root quaternion/translation ordering to XYZ + xyzw and names all joints.
Changing to the ball-hand asset can introduce contact errors; it does not refine
hand positions. Existing results: [`data/omniretarget`](../../data/omniretarget).

## Reproduce the complete climbing selection

The library now includes all 29 released `climb_00`–`climb_28` trajectories.
Seven were previously present; the ACP-source update adds the other 22 without
changing the earlier files. Their exact membership in ACP's training set is unknown.

```bash
uv run python data_generator/omniretarget/download.py --output vendor/omniretarget
uv run python data_generator/build_acp_library.py --sources omniretarget \
  --omniretarget_source vendor/omniretarget --output_root outputs/acp_rebuild \
  --isaaclab_python /path/to/IsaacLab/.venv/bin/python \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml
```

`download.py` pins the dataset revision and downloads the robot-terrain archive,
source robot joint labels, and referenced terrain models. `build_acp_library.py`
uses `acp_sources.json` to select the 22 new clips, exports the exact ball-hand USD
body states, normalizes terrain keys, screens joint limits/foot geometry, and
records a complete video. Logs go under ignored `outputs/acp_logs/`.
`--resume` reuses completed stages for an unchanged request.

Joint-limit or contact-screening failures remain visible in `validation.json`.
No clip is marked training-ready merely because its NPZ loads successfully.
