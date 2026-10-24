# Mixamo: local FBX importer

ACP cites Mixamo for stair climbing, ladder climbing, and sitting. Mixamo requires
an Adobe account; no downloaded Mixamo files were available during this update.
This directory prepares an importer for your own FBX exports. No Mixamo motion is
bundled, and this is not ACP's retargeting implementation.

## Install and import

The optional `bpy` wheel provides Blender's FBX importer and requires Python 3.11.
Keep it in a separate uv environment so the base Python 3.12 environment is unchanged:

```bash
UV_PROJECT_ENVIRONMENT=.venv-mixamo uv sync --python 3.11 --locked --extra mixamo
UV_PROJECT_ENVIRONMENT=.venv-mixamo uv run --python 3.11 --extra mixamo \
  python data_generator/mixamo/import_mixamo.py \
  --input /path/to/ClimbingLadder.fbx \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml \
  --output outputs/mixamo/ladder
```

Export an FBX with its animated Mixamo skeleton. The importer requires Hips,
Spine2, and the standard Left/Right UpLeg, Leg, Foot, Arm, ForeArm, and Hand bones;
namespace prefixes such as `mixamorig:` are accepted. It reads bind transforms and
samples the animation in meters/Z-up. `--animation_index` selects an imported
action when the file contains several animations. The default sampling rate is 30 Hz.

The baseline uses constrained Mink IK for the root, torso, knees, and ankles.
Arm IK matches upper-arm/forearm directions rather than copying incompatible joint
angles. Source wrist motion is omitted and target wrists stay neutral. It writes
`g1_qpos.npz` and provenance; use the shared USD exporter and player afterward.

## Terrain and limitations

`--terrain terrain.json` accepts canonical box/mesh fields from `data/README.md`.
Coordinates must already be in the target G1 reference frame, after motion scaling
and the initial root XY translation. No terrain is inferred from the FBX.
Without this option, the output only declares a flat plane at Z=0; that does not
make a ladder/stair/sitting animation terrain-compatible. Inspect the reported
scale, robot geometry, and intended contacts before constructing a tracking task.

The FBX path and IK are checked with a synthetic animated skeleton. Real Mixamo
naturalness, contact fit, and dynamic tracking remain untested until a real export
is supplied. This importer is a starting point, not a validated climbing dataset.

[Adobe's FAQ](https://community.adobe.com/questions-696/mixamo-faq-licensing-royalties-ownership-eula-and-tos-589400?lang=en)
restricts redistribution of animation files and also lists restrictions on machine
learning use. The importer does not establish permission for training. Use the
terms applicable to your own download. Local exports in `data/mixamo/<clip>/` are
ignored by Git and can be inspected through `tools/build_catalog.py --include-local`.
