# CUCR museum → Isaac USD

Source: `CardiffUniversityComputationalRobotics/cucr_worlds` / `cucr_worlds_museum`.

Uses **Isaac `omni.kit.asset_converter`** (must `await task.wait_until_finished()`; polling
`is_finished()` alone never completes). Assimp is only used as a Collada→OBJ front-end.

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

Scenario: `src/scenarios/museum_agents.yaml`.
