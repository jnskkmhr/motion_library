# PARC retargeting

Human motion and terrain source: [mxucg/PARC](https://huggingface.co/datasets/mxucg/PARC).
The five running-jump clips use the corrected `parc_anatomical_arms_v2` conversion.

```bash
uv run python data_generator/parc/retarget_parc.py \
  --motion /path/to/terrain_run_jump_gap_005_dm.pkl \
  --source_xml /path/to/PARC/humanoid.xml \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml \
  --output outputs/parc/jump_005
```

The root/torso/legs use Mink IK. Arm IK matches the source upper-arm and forearm
directions in the torso frame; elbow motion is not copied from incompatible joint
coordinates. Wrists stay neutral. Motion and terrain share a uniform spatial scale;
source timing is unchanged. The MJCF must retain its referenced meshes.

The output `g1_qpos.npz` is an intermediate. Use the [shared USD exporter](../shared/README.md)
for the final named-body mimic NPZ. Existing results: [`data/parc`](../../data/parc),
including arm and gap measurements. Foot-contact correction is still needed;
these running leaps are not validated tracking-policy executions.
