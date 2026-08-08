# CUCR hospital to Isaac USD

Source: `CardiffUniversityComputationalRobotics/cucr_worlds` / `cucr_worlds_hospital`
(branch `gz_humble`). Gazebo world is a replica of
[aws-robomaker-hospital-world](https://github.com/aws-robotics/aws-robomaker-hospital-world).

**Not** NVIDIA Isaac `Environments/Hospital` (that was a wrong interim path).

## v1 scope

- Building: `aws_robomaker_hospital_floor_01_floor` + `_walls` (+ nurses station)
- Occupancy: CUCR `maps/hospital.pgm` / `hospital.yaml` (`origin: [-12.5, -35]`)
- Props from `hospital.world` deferred (beds, carts, …)

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

Outputs:
- `src/worlds/assets/hospital/hospital_{floor,walls,nursesstation}.usd`
- `src/worlds/hospital.usd` (composed Z-up stage + static colliders)

**Look / washout:** Assimp MTL `Ke` becomes `UsdPreviewSurface` `emissiveColor` (walls glow).
The convert tool zeros emissive after convert and uses `DistantLight` intensity **1800**
(not museum’s 5000). White AWS hospital needs that dimmer light for textures to read.

Scenarios must use the **CUCR** map frame (not HuNav Isaac-stock goals).
