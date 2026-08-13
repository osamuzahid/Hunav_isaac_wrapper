# CUCR bookstore to Isaac USD

Source: `CardiffUniversityComputationalRobotics/cucr_worlds` / `cucr_worlds_bookstore`
(branch `gz_humble`). AWS RoboMaker retail shop (~15 m). Ceiling slab skipped.

## Scope (1:1 minus ceiling)

- Floor, walls, door, shop window, bookshelves, desks, chairs, books, tablets.
- Skip `aws_robomaker_retail_RetailShopCeiling_01` (opaque top-down).
- Skip Gazebo people if present — HuNav + Stretch replace those.
- DAE is Z-up, `<unit meter="0.01">`. Assimp OBJ is already in metres (Y-up).
  Compose scale **1.0**. Converter `convert_stage_up_z=True` only sets USD
  upAxis; compose adds **RotateX(90)** so Y-up meshes sit on a Z-up floor.
- Occupancy: CUCR `maps/map.pgm` cropped to the shop
  (`origin ≈ [-9.95, -9.40]`, res `0.05`).

## Convert

Do **not** run this while the Isaac GUI is up (16 GB host).

```bash
git clone --filter=blob:none --sparse --depth 1 -b gz_humble \
  https://github.com/CardiffUniversityComputationalRobotics/cucr_worlds.git \
  /tmp/cucr_bookstore_src/cucr_worlds
cd /tmp/cucr_bookstore_src/cucr_worlds
git sparse-checkout set cucr_worlds_bookstore

cd /path/to/Hunav_isaac_wrapper
python3 tools/prepare_bookstore.py \
  --cucr-root /tmp/cucr_bookstore_src/cucr_worlds \
  --out /tmp/cucr_bookstore_src/obj

OMNI_KIT_ACCEPT_EULA=YES HUNAV_BOOKSTORE_OBJ_DIR=/tmp/cucr_bookstore_src/obj \
  ~/isaacsim/python.sh tools/isaac_convert_bookstore.py
```

`--compose-only` / `HUNAV_BOOKSTORE_COMPOSE_ONLY=1` rewrites lighting and
payload refs without converting meshes.

Outputs:

- `src/worlds/assets/bookstore/*.usd` + `textures/`
- `src/worlds/bookstore.usd` (composed stage, absolute payload refs)
- `src/maps/bookstore.png` + `bookstore.yaml`

Offline routes:

```bash
python3 tools/plan_bookstore_routes.py
```

Launcher: double-click `~/Desktop/Run Bookstore Behaviors.desktop` **once**
(`--world bookstore --config bookstore_behaviors`). Do not start a second Isaac
while the first is coming up. New `.desktop` files need GNOME
`metadata::trusted` (otherwise a red X and the filename as the label).

GUI: operator-confirmed 2026-08-13 (Validated **#58**).

Shared checklist: `isaac-social-nav/docs/CUCR_WORLD_PORT.md`.

## Lessons applied from office (#55–#57)

- Keep SDF / Collada scales. Assimp already baked cm→m into OBJ.
- No ceiling slab.
- Absolute PNG paths on asset USDs; disconnect opacity unless the PNG has alpha.
- `embed_textures=False`.
- `setup.py` installs `worlds/assets/bookstore/` (+ `textures/`).
- Compose payload refs as absolute paths. `find_package_share_directory` prefers
  source `src/worlds`.
- DistantLight 5000 + DomeLight 400.
- Mixed HuNav BTs only on disjoint aisle loops (west-north, east seating,
  west-south, north door, south aisle). Stretch spawn `(0.2, 1.0)`.
