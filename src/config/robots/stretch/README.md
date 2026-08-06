# Hello Robot Stretch (SE3)

Vendored URDF + meshes for `--robot stretch` in the HuNav Isaac wrapper.

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

## Runtime

Chassis-only kinematic drive: `/cmd_vel` moves the root Xform; arm DOFs are not commanded.

Optional spawn pose from scenario YAML:

```yaml
hunav_loader:
  ros__parameters:
    robot_init_pose: {x: -4.0, y: -3.0, z: 0.0, h: 0.0}
```
