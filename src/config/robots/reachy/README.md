# Reachy 2023 (Pollen) — vendored for isaac-social-nav

Apache-2.0 (Pollen Robotics). Source: official
[`pollen-robotics/reachy_2023`](https://github.com/pollen-robotics/reachy_2023)
`reachy_description` (develop). **Torso-only** kit (Zuuu mobile-base xacro stripped for Phase 0).
Camera optical frames come from the description’s Gazebo camera section (links kept; plugins stripped).

## Stock sensors (Phase 0)

| Stream | Topic | Notes |
|---|---|---|
| TF | `/tf` | parked rclpy: `world`→`torso` / camera opticals (PoseTree unusable on opticals) |
| Joint states | `/joint_states` | parked zeros via rclpy; bodies **`park_kinematic`** (dynamic PhysX blows on bad upstream inertias) |
| Head RGB L/R | `/left_camera/*`, `/right_camera/*` | opt-in `HUNAV_LAB_CAMERAS=1`; no gripper force |

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
  --fix-base --merge-mesh
```

Baked `.obj`/`.mtl` are gitignored (large); keep source `.dae` and re-bake before
re-import. `orbita_arm.dae` material has `name="orbita_arm_mat"` (Isaac rejects
unnamed Collada materials).

**PhysX note:** upstream `shoulder_x` inertias are ~`1e4` (CAD garbage) and blow
the articulation apart under gravity (vertical “exploded” stack). Vendored xacro
uses cylinder-scale inertias; spawn uses `park_kinematic=True` so the parked lab
pose stays assembled.