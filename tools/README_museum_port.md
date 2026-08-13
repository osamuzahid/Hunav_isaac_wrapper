# CUCR museum to Isaac USD

Source: `CardiffUniversityComputationalRobotics/cucr_worlds` / `cucr_worlds_museum`.

Uses Isaac `omni.kit.asset_converter` (must `await task.wait_until_finished()`; polling
`is_finished()` alone never completes). Assimp is only used as a Collada-to-OBJ front-end.

```bash
# 1) OBJ + textures beside them
mkdir -p /tmp/cucr_museum_src/obj
assimp export path/to/new_museum.dae /tmp/cucr_museum_src/obj/museum.obj
assimp export path/to/floor.dae /tmp/cucr_museum_src/obj/floor.obj
cp path/to/museum/meshes/*.{png,jpeg,jpg} /tmp/cucr_museum_src/obj/ 2>/dev/null || true

# 2) Isaac convert + compose
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/isaac_convert_museum.py
```

Outputs:
- `src/worlds/assets/museum/museum_mesh.usd` (+ `textures/`)
- `src/worlds/assets/museum/museum_floor.usd`
- `src/worlds/museum.usd` (composed stage, Z-up wrapper + colliders)

Intentional differences from CUCR Gazebo (documented in-script as
`ORIGINALLY` / `PATCH (isaac-social-nav)` and in
`src/worlds/assets/museum/PATCH_NOTES.md`): light brown floor instead of neon
blue; brighter distant light; Z-up compose rotate; museum mesh Z **0.1** (not
Gazebo visual 0.5) so walls meet the floor and base lidar hits them.

Scenario: `src/scenarios/museum_agents.yaml`.

Next CUCR worlds: `isaac-social-nav/docs/CUCR_WORLD_PORT.md`.
