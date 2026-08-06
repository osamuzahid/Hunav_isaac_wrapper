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

Wheel params (from URDF): radius `0.0508` m, wheel base `0.3407` m (`2 × 0.17035`).

Optional spawn pose from scenario YAML:

```yaml
hunav_loader:
  ros__parameters:
    robot_init_pose: {x: -4.0, y: -3.0, z: 0.0, h: 0.0}
```

`stretch_wheeled` applies a tiny `spawn_z_lift` (2 mm) so tire spheres settle onto the ground instead of spawning in penetration.
