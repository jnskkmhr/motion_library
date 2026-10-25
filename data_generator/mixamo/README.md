# Mixamo: FBX importer and G1 video preview

ACP cites Mixamo for stair climbing, ladder climbing, and sitting. The gallery now shows a real Mixamo Samba Dancing sample from the public Three.js
examples, converted with this importer. This is not ACP's retargeting implementation
or its stair/ladder/sitting data. Only the rendered video is published; source FBX
and converted NPZ arrays stay local.

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

The FBX path and IK were checked with a synthetic skeleton and the real Samba
Dancing sample. The latter shows foot sliding and up to 6.9 cm of foot penetration
below the ground plane. This is an importer demonstration, not a validated training
or climbing dataset. Dynamic tracking has not been tested.

[Adobe's FAQ](https://community.adobe.com/questions-696/mixamo-faq-licensing-royalties-ownership-eula-and-tos-589400?lang=en)
restricts redistribution of animation files and also lists restrictions on machine
learning use. The importer does not establish permission for training. Use the
terms applicable to your own download. Local exports in `data/mixamo/<clip>/` are
ignored by Git and can be inspected through `tools/build_catalog.py --include-local`.

## View the sample

[Public G1 video](https://jnskkmhr.github.io/motion_library/docs/#mixamo%2Fsamba_dancing).
Select **Mixamo → Samba dancing**. The public entry links to its source rather than
advertising an unavailable NPZ download. The local gallery includes the converted NPZ.

The sample was downloaded from:
`https://raw.githubusercontent.com/mrdoob/three.js/r180/examples/models/fbx/Samba%20Dancing.fbx`.
Its SHA-256 is recorded in `docs/previews/mixamo_samba/provenance.json`.

To reproduce it, download that FBX, then use the import command above with
`--output data/mixamo/samba_dancing`. Export and record using your Isaac Lab Python:

```bash
/path/to/IsaacLab/.venv/bin/python data_generator/shared/export_mimic.py \
  --input data/mixamo/samba_dancing/g1_qpos.npz \
  --output data/mixamo/samba_dancing/motion_raw.npz \
  --robot_asset UNITREE_G1_29DOF_BALL_HAND_CFG
uv run python tools/normalize_terrain.py \
  --input data/mixamo/samba_dancing/motion_raw.npz \
  --output data/mixamo/samba_dancing/motion.npz --ground_height 0
uv run --no-project --python /path/to/IsaacLab/.venv/bin/python \
  --with imageio --with imageio-ffmpeg python integrations/isaaclab/play_motion_file.py \
  --motion_file data/mixamo/samba_dancing/motion.npz \
  --video --headless --video_folder data/mixamo/samba_dancing \
  --video_resolution 960 640 --camera_offset -1.5 -2.5 0.8
uv run python tools/build_catalog.py --include-local
```

Open `docs/local.html` to inspect the local video and download the converted NPZ.
The public video, provenance, and screening report live in `docs/previews/mixamo_samba/`.
