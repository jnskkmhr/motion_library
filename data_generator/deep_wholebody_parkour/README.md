# Deep Whole-body Parkour import

Imports released G1 references and scanned terrain from the
[official project](https://project-instinct.github.io/deep-whole-body-parkour/).
The source G1 uses a torso-rooted URDF; this importer reconstructs the pelvis pose
and reverses the three waist joint angles to match the target chain.

```bash
uv run python data_generator/deep_wholebody_parkour/import_deep_parkour.py \
  --motion /path/to/rollVault11-ziwen-retargeted.npz \
  --terrain /path/to/50cmboxes_noWall.stl \
  --source_urdf /path/to/g1_torso.urdf \
  --mjcf /path/to/newton-assets/unitree_g1/mjcf/g1_29dof_rev_1_0_box_foot_ball_hand.xml \
  --output outputs/deep_wholebody_parkour/roll_vault
```

The importer retains source timing and terrain vertices and records source/target
forward-kinematics differences. Use the [shared USD exporter](../shared/README.md)
after import. The existing dive-roll, jump-sit, kneel-climb, and roll-vault results
are in [`data/deep_wholebody_parkour`](../../data/deep_wholebody_parkour).
Hand and foot intersections remain in some clips; inspect their reports.
