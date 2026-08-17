# Remaining CUCR worlds → Isaac USD

Ports of `cucr_worlds_house_museum`, `cucr_worlds_small_house`, and
`cucr_worlds_small_warehouse` (`gz_humble`). Shared checklist:
`isaac-social-nav/docs/CUCR_WORLD_PORT.md`.

`--world warehouse` remains the **stock HuNav** bake. CUCR warehouse is
`--world small_warehouse`. `--world museum` is the CUCR gallery; house museum
is `--world house_museum`.

## Convert

Do **not** run this while the Isaac GUI is up (16 GB host).

```bash
git clone --filter=blob:none --sparse --depth 1 -b gz_humble \
  https://github.com/CardiffUniversityComputationalRobotics/cucr_worlds.git \
  /tmp/cucr_remaining_src/cucr_worlds
cd /tmp/cucr_remaining_src/cucr_worlds
git sparse-checkout set cucr_worlds_house_museum cucr_worlds_small_house \
  cucr_worlds_small_warehouse

cd /path/to/Hunav_isaac_wrapper
for w in house_museum small_house small_warehouse; do
  python3 tools/prepare_cucr_world.py --world "$w" \
    --cucr-root /tmp/cucr_remaining_src/cucr_worlds \
    --out /tmp/cucr_remaining_src/obj/$w
  OMNI_KIT_ACCEPT_EULA=YES HUNAV_CUCR_OBJ_DIR=/tmp/cucr_remaining_src/obj/$w \
    ~/isaacsim/python.sh tools/isaac_convert_cucr_world.py --world "$w"
done
```

`--compose-only` / `HUNAV_CUCR_COMPOSE_ONLY=1` rewrites lighting and payload
refs without converting meshes.

Offline routes:

```bash
python3 tools/plan_cucr_routes.py --world house_museum
python3 tools/plan_cucr_routes.py --world small_house
python3 tools/plan_cucr_routes.py --world small_warehouse
```

## Per-world notes

| World | Source | Skip | Occupancy | Stretch spawn |
|---|---|---|---|---|
| `house_museum` | single `house_museum.dae` (Z-up Collada; Assimp OBJ is Y-up) | — | cropped gmapping 4000² → origin `[-11.80, -8.30]` | `(-2.1, 0.0)` |
| `small_house` | AWS residential wrapping `<model>` poses | `RoomCeiling_01` | `[-10, -10]`, 0.05 | `(0.0, 0.0)` |
| `small_warehouse` | AWS warehouse wrapping `<model>` poses | `RoofB_01` | `[-6.95, -10.40]`, 0.05 | `(0.0, 0.0)` |

House museum compose keeps `z=0.1` so wall bottoms meet the floor / base lidar
(same class as museum **#47**). SDF comments are stripped before parsing
(warehouse `GroundB` floor lived after a commented DeskC block).

**MTL `#` names:** Assimp writes `newmtl Material #112`. MTL treats `#` as a
comment, so Isaac's converter drops the UsdUVTexture graph (`uvtex=0`) and the
mesh renders clay-grey. Prepare rewrites those to `Material_112`. Pallet-jack
(`Car_body`) and chair materials without `#` already converted.

**house_museum:** keep the DAE sunset sky (`material.001` + DomeLight JPEG).
Strip **roof/ceiling slabs only** (house wing y≈2.1 m; gallery higher) so top-down
can see interiors. Do not strip sky faces. Sampler stub
`WoodBaked_baseColor_jpeg--sampler.jpg` → real JPEG. Sky mesh double-sided,
collision off.

Launcher: double-click the matching `~/Desktop/Run *.desktop` **once**.
New `.desktop` files need GNOME `metadata::trusted`.

**GUI (2026-08-17):** small_house and small_warehouse operator-good; house_museum
operator-good with roofs stripped and CUCR sunset sky kept.

Shared lessons (office **#55–#57**, bookstore **#58**): keep SDF/Collada scales;
no ceiling slab; absolute texture paths; `embed_textures=False`; compose
RotateX(90) for Assimp Y-up OBJs; mixed HuNav BTs only on disjoint loops.
