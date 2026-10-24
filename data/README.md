# Dataset contract

Each gallery motion has `motion.npz` and `motion.mp4`. Reports and intermediate
qpos files are preserved when available. `motion_generated.*` is an optional
version before contact refinement. The catalog indexes final motion/video pairs;
intermediate `g1_qpos.npz` files are not mimic training files.

All bundled final references use 50 FPS, 29 named joints, and 35 body frames for
the ball-hand G1. Duration in the gallery is frame count / FPS; the last sample
time is (frame count − 1) / FPS. Body quaternions are xyzw. Positions are meters;
joint angles are radians; velocities are per second.

| Key | Shape / meaning |
| --- | --- |
| `fps` | Scalar sampling frequency |
| `joint_names`, `body_names` | Column labels |
| `joint_pos`, `joint_vel` | `(T, 29)` |
| `body_pos_w`, `body_lin_vel_w`, `body_ang_vel_w` | `(T, 35, 3)`; link origins in the reference frame |
| `body_quat_w` | `(T, 35, 4)` |
| `terrain_position` | `(N, 3)` box centers in the same reference frame |
| `terrain_dimension` | `(N, 3)` full local-axis box sizes |
| `terrain_quat` | `(N, 4)` box orientations, xyzw; optional identity |
| `terrain_vertices`, `terrain_faces` | Triangle mesh vertices and indices |
| `ground_height` | Optional ground top Z in the reference frame |
| `robot_asset` | Required asset choice for these exports |

Ground height is explicit so a global plane does not fill a jump gap. New clips
must keep terrain and motion in the same coordinate frame. Training translates
both by the same terrain-patch offset. Do not independently randomize reference
XY/yaw while leaving terrain fixed.

See the [catalog](catalog.json), [extraction manifest](import_manifest.json), and
per-clip provenance. Catalog checksums apply to the current canonical NPZ, not
the original upstream file. The metadata normalization does not change motion
arrays. Videos are kinematic visualization, not evidence of stable control.
