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
