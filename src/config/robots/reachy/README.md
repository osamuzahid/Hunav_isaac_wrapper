# Reachy 2023 (Pollen) — vendored for isaac-social-nav

Apache-2.0 (Pollen Robotics). Source: official
[`pollen-robotics/reachy_2023`](https://github.com/pollen-robotics/reachy_2023)
`reachy_description` (develop) **plus Zuuu mobile base** (`mobile_base_visual.dae`,
lidar at `lidar_link`). Camera optical frames come from the description’s Gazebo
camera section (links kept; plugins stripped). Wheels are **fixed** — planar
motion is kinematic `ChassisDriveRobot` (`/cmd_vel` `vx`+`wz`), not PhysX omni drive.

## Stock sensors

| Stream | Topic | Notes |
|---|---|---|
| TF | `/tf` | Nav2 tree: `world→map→odom→base_link→lidar_link` (`laser` alias of the URDF pin). RELIABLE. |
| Odom | `/odom` | Ground-truth chassis pose (BEST_EFFORT) |
| Joint states | `/joint_states` | parked zeros via rclpy (arm visual-only) |
| Base lidar | `/scan` | RTX 2D on `lidar_link` (frame_id=`lidar_link`) |
| IMU | `/imu` | synthetic gravity on `imu_link` |
| Head RGB L/R | `/left_camera/*`, `/right_camera/*` | opt-in `HUNAV_LAB_CAMERAS=1`; no gripper force |

Drive is **kinematic chassis** (`--robot reachy`, same `ChassisDriveRobot` as Stretch). `/cmd_vel` `linear.x` + `angular.z`. Last twist is **held** until a new message (including zeros). Do **not** PhysX omniwheels. Raw `/cmd_vel` **ghosts walls**; occupancy / Nav2 / ESC keep it in halls. Hospital spawn `(5.0, 0.0)` yaw `2.9`. Occupancy hop (B3): `(5, 0)` → `(5, -8)` south hall (`tools/drive_reachy_waypoints.py`).

RViz `reachy_scan.rviz`: Fixed Frame `lidar_link` for a local scan check. TF **Show Arrows** draws a grey dotted line from the sensor — that is not a lidar ray.

## Rebuild USD

Collada (`.dae`) multi-node scenes import “scattered” in Isaac (geometry far from
link origins). Bake with **`-ptv`** (pretransform vertices) so each OBJ is a
single mesh in the Collada root / link visual frame. Assimp’s OBJ export is
**Y-up** even when the DAE says `Z_UP` — apply **Rx(+90°)** `(x,y,z)→(x,-z,y)`
after bake so arm shells align with the −Z joint chain (without this, white
housings stack as a vertical “exploded” look).

```bash
# from Hunav_isaac_wrapper/
./tools/bake_reachy_meshes.sh
# regenerates reachy.urdf (.obj mesh refs) then:
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/isaac_import_robot_urdf.py \
  --urdf src/config/robots/reachy/reachy.urdf \
  --usd-path src/config/robots/reachy --output-name reachy.usd \
  --no-fix-base --merge-mesh
```

Baked `.obj`/`.mtl` are gitignored (large); keep source `.dae` and re-bake before
re-import. `orbita_arm.dae` material has `name="orbita_arm_mat"` (Isaac rejects
unnamed Collada materials).

**PhysX note:** upstream `shoulder_x` inertias are ~`1e4` (CAD garbage) and blow
the articulation apart under gravity (vertical “exploded” stack). Vendored xacro
uses cylinder-scale inertias; spawn uses `park_kinematic=True` so the parked lab
pose stays assembled.