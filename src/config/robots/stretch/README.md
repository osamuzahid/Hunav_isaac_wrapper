# Hello Robot Stretch (SE3)

Vendored URDF + meshes for `--robot stretch` / `--robot stretch_wheeled` in the HuNav Isaac wrapper.

## Source & license

- **URDF/meshes:** `hello-robot-stretch-urdf` pip package (Hello Robot Inc., Clear BSD — see `LICENSE.md`).
- **Upstream:** [hello-robot/stretch_urdf](https://github.com/hello-robot/stretch_urdf) / Cardiff `stretch_ros2` `stretch_description` lineage.
- Model: **SE3** with `eoa_wrist_dw3_tool_sg3` gripper.

## USD generation

```bash
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/isaac_import_robot_urdf.py
```

Output: `stretch.usd` + `payloads/` (Isaac 6.0 URDF importer).

### Isaac 6.0 importer notes (PATCH isaac-social-nav)

The stock Stretch URDF triggers Isaac 6.0 `getPrimNames()` failures when:

1. Visual `<material name="">` tags are empty, or
2. The RealSense `d435.dae` Collada mesh is referenced (unnamed Collada materials).

`stretch.urdf` is pre-patched: empty/inline materials stripped, `d435.dae` head-camera visual removed (arm/base/gripper meshes kept). Re-run the import tool after editing the URDF.


### PhysX drive notes (PATCH isaac-social-nav)

Root cause of “wheels spin, base barely moves”:

1. **Floor sled** — `base_link_collision` STL reaches `z=0`, so the chassis sat on the ground and drive wheels had no normal load.
2. **Bad rollers** — 12-triangle convexHull wheel meshes are poor contact geometry for rolling.
3. **Fixed caster scrub** — `caster_joint` is URDF-`fixed`, so the caster sphere cannot roll; any contact friction acts as a rear brake.
4. **Extra hulls** — other imported mesh colliders (and an overlapping chassis Cube) jammed the articulation under PhysX.

Fixes applied:

- Removed `instanceable = true` from `payloads/base.usda` (blocked `WheeledRobot` DOF writes).
- Zeroed `physxJoint:jointFriction` on wheel joints; raised angular drive damping/`maxForce` in `payloads/Physics/physx.usda`.
- Disabled PhysX articulation self-collisions.
- Disabled all imported `convexHull` mesh colliders in `payloads/instances.usda`.
- Added sphere tire colliders (`radius=0.0508`) with friction on left/right wheels.
- Disabled caster sphere collision (fixed joint cannot roll).
- Disabled chassis floor mesh + chassis box; wall contact via elevated `mast_wall_collision` cylinder on `link_mast` (plus existing camera boxes).

Headless check:

```bash
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/stretch_wheeled_smoke.py
```

## Runtime — choose drive mode via `--robot`

| `--robot` | Drive | Walls | Teleop |
|---|---|---|---|
| `stretch` | Kinematic chassis (`set_world_pose` from `/cmd_vel`); arm visual-only | No | **Works** (default) |
| `stretch_wheeled` | `WheeledRobot` + `DifferentialController` on wheel joints | Yes (mast/camera colliders) | **Works** (PhysX traction) |

## Stock sensors (PATCH isaac-social-nav)

Attached at runtime by `lab_robot_sensors.py` (not from URDF optics):

| Stream | Topic | Notes |
|---|---|---|
| TF | `/tf` | base + laser + imu + camera frames |
| Joint states | `/joint_states` | parked zeros when `Physics=none`; OmniGraph when wheeled |
| Lidar | `/scan` | RTX `Example_Rotary_2D` under `Geometry/base_link/laser`. Stock NVIDIA profile aims **−2°**; we set `emitterState:*/elevationDeg` to **0°** so the beam matches a real SE3 RPLidar (horizontal in the `laser` frame) — not a prim pitch hack. Museum sparse “feet” was **world Z** (walls floated): keep `MUSEUM_Z_OFFSET=0.1` — see main-repo TROUBLESHOOTING 2026-08-12 / Validated **#47**. |
| IMU | `/imu` | synthetic gravity-only (static/kinematic bases) |
| RGB-D | `/camera/color/*`, `/camera/depth/*` | opt-in: `HUNAV_LAB_CAMERAS=1`; Camera under `camera_color_optical_frame` with **fixed** optical→OpenGL `Rx(180)·Rz(−90)` (reset-safe). Do **not** parent under `camera_link` — parked head_tilt origin makes link +X ≠ RealSense look. Kinematic Stretch uses parked rclpy `/tf` (PoseTree `eInvalid` on `Physics=none`). See main-repo TROUBLESHOOTING / Validated **#46** / **#50**. |

Disable all: `HUNAV_LAB_SENSORS=0`. Smoke: `tools/lab_robot_sensor_smoke.py --robot stretch`
(checks topic presence **and** TF frames `base_link`/`laser`/`base_imu`, parked joint
names, IMU gravity ≈9.81, `/scan` beam array).

RGB-D GUI check (separate ROS shell once sim is up):

```bash
ros2 run rqt_image_view rqt_image_view   # topic: /camera/color/image_raw
```

Lidar GUI check (museum wall rings after `#47`):

```bash
HUNAV_LAB_LIDAR=1 HUNAV_LAB_CAMERAS=0 \
  ros2 run hunav_isaac_wrapper hunav_isaac_launcher \
  --debug --no-headless --batch --robot stretch --world museum --config museum_sensor_demo
# separate shell:
ros2 run rviz2 rviz2   # Fixed Frame: laser; LaserScan topic: /scan
```
Wheel params (from URDF): radius `0.0508` m, wheel base `0.3407` m (`2 × 0.17035`).

Optional spawn pose from scenario YAML:

```yaml
hunav_loader:
  ros__parameters:
    robot_init_pose: {x: -4.0, y: -3.0, z: 0.0, h: 0.0}
```

`stretch_wheeled` applies a tiny `spawn_z_lift` (2 mm) so tire spheres settle onto the ground instead of spawning in penetration.
