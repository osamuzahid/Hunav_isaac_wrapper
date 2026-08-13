# CUCR hospital to Isaac USD

Source: `CardiffUniversityComputationalRobotics/cucr_worlds` / `cucr_worlds_hospital`
(branch `gz_humble`). Gazebo world is a replica of
[aws-robomaker-hospital-world](https://github.com/aws-robotics/aws-robomaker-hospital-world).

**Not** NVIDIA Isaac `Environments/Hospital` (that was a wrong interim path).

## Scope

- Building: `aws_robomaker_hospital_floor_01_floor` + `_walls` (+ nurses station)
- Occupancy: CUCR `maps/hospital.pgm` / `hospital.yaml` (`origin: [-12.5, -35]`)
- **Props:** Gazebo `hospital.world` furniture/equipment (chairs, carts, curtains, …)
  via `prepare_hospital_props.py` + `isaac_convert_hospital.py --props`

## Convert

```bash
# Sparse clone (example)
git clone --filter=blob:none --sparse --depth 1 -b gz_humble \
  https://github.com/CardiffUniversityComputationalRobotics/cucr_worlds.git /tmp/cucr_hospital_src/cucr_worlds
cd /tmp/cucr_hospital_src/cucr_worlds
git sparse-checkout set \
  cucr_worlds_hospital/maps \
  cucr_worlds_hospital/models/aws_robomaker_hospital_floor_01_floor \
  cucr_worlds_hospital/models/aws_robomaker_hospital_floor_01_walls \
  cucr_worlds_hospital/models/aws_robomaker_hospital_nursesstation_01

# Assimp DAE→OBJ (strip leading whitespace after <color> tags if Assimp errors)
mkdir -p /tmp/cucr_hospital_src/obj
# … export floor.obj walls.obj nursesstation.obj + copy *.png …

OMNI_KIT_ACCEPT_EULA=YES HUNAV_HOSPITAL_OBJ_DIR=/tmp/cucr_hospital_src/obj \
  ~/isaacsim/python.sh tools/isaac_convert_hospital.py

# Install CUCR map into wrapper maps/
cp /tmp/cucr_hospital_src/cucr_worlds/cucr_worlds_hospital/maps/hospital.pgm \
   src/maps/hospital.png   # or keep .pgm if loader accepts; we copy as png-compatible PGM
cp /tmp/cucr_hospital_src/cucr_worlds/cucr_worlds_hospital/maps/hospital.yaml \
   src/maps/hospital.yaml
```

### Props (furniture / equipment)

```bash
# Sparse-checkout models + worlds (see prepare script defaults under /tmp/cucr_hospital_src)
python3 tools/prepare_hospital_props.py \
  --cucr-root /tmp/cucr_hospital_src/cucr_worlds \
  --out /tmp/cucr_hospital_src/obj_props

# Reuse existing building USDs; convert unique prop meshes + recompose
OMNI_KIT_ACCEPT_EULA=YES HUNAV_HOSPITAL_PROPS_OBJ_DIR=/tmp/cucr_hospital_src/obj_props \
  ~/isaacsim/python.sh tools/isaac_convert_hospital.py --props
```

Outputs:
- `src/worlds/assets/hospital/hospital_{floor,walls,nursesstation}.usd`
- `src/worlds/assets/hospital/props/*.usd` (unique models)
- `src/worlds/hospital.usd` (building + `/World/hospital_props/*` + static colliders)

Compose applies Gazebo `model.sdf` `<mesh><scale>` (chairs/IV stands are ~0.008).
`floor_01_ceiling` is skipped so top-down views stay usable.

**GUI Carter:** keepalive sets `Configuration=Full_Merged` and expands body visual
instances; follow `/World/Nova_Carter/chassis_link` (not the empty root Xform).
Desktop helper: `~/Desktop/run-hospital-nav2-smoke.sh` (`--frame-robot`).

**Nav2 smoke (after keepalive `--world hospital --config hospital_agents`):**
`./tools/nav2_smoke/run_hospital_nav2_smoke.sh`  
Default goal `(8.4,-18.8)` ≈ 2 m along `robot_init_pose` yaw. Avoid `(10,-17)` —
CUCR map obstacles near `y≈-18.8` plus `inflation_radius: 0.55` make that plan fail.

**Look / washout:** Assimp MTL `Ke` becomes `UsdPreviewSurface` `emissiveColor` (walls glow).
The convert tool zeros emissive after convert and uses `DistantLight` intensity **1800**
(not museum’s 5000). White AWS hospital needs that dimmer light for textures to read.

Scenarios must use the **CUCR** map frame (not HuNav Isaac-stock goals).

Next CUCR worlds: follow `isaac-social-nav/docs/CUCR_WORLD_PORT.md` (scales, ceilings,
asset install path, texture refs).
