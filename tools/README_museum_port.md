# CUCR museum → Isaac USD

Source: `cucr_worlds` / `cucr_worlds_museum`.

```bash
assimp export path/to/new_museum.dae /tmp/cucr_museum_src/obj/museum.obj
assimp export path/to/floor.dae /tmp/cucr_museum_src/obj/floor.obj
OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/obj_to_museum_usd.py
```

Outputs: `src/worlds/museum.usd` + `src/worlds/assets/museum/`.
Scenario: `src/scenarios/museum_agents.yaml`.
