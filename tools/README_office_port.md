# CUCR office to Isaac USD

Source: `CardiffUniversityComputationalRobotics/cucr_worlds` / `cucr_worlds_office`
(branch `gz_humble`). AWS ServiceSim small-office. **Not** the stock HuNav
`office.usd` bake.

## Scope (1:1)

- Floor plane (Hallway texture) and SDF wall boxes with `plain.png` / elevator /
  door materials. **No ceiling** (opaque slab hid top-down views; same as hospital).
- Room, cubicle, door, carpet, tile meshes (`scale 0.01` — do not drop; hospital **#41**).
- Nested furniture includes (desks `scale 0.001`, chairs, computers, cafe, etc.).
- Occupancy raster from wall boxes only (`origin ≈ [-27.72, 0]`, res `0.05`).
  Carpet/tile are visual-only (no collision).
- Skip Gazebo people (`casual_*` / `elegant_*` / `actor`) and `servicebot` —
  HuNav + Stretch replace those.
- Stock HuNav office: `office.usd.bak_hunav_isaac_stock` + `maps/office_isaac_stock.*`.

## Convert

```bash
git clone --filter=blob:none --sparse --depth 1 -b gz_humble \
  https://github.com/CardiffUniversityComputationalRobotics/cucr_worlds.git \
  /tmp/cucr_office_src/cucr_worlds
cd /tmp/cucr_office_src/cucr_worlds
git sparse-checkout set cucr_worlds_office

cd /path/to/Hunav_isaac_wrapper
python3 tools/prepare_office.py \
  --cucr-root /tmp/cucr_office_src/cucr_worlds \
  --out /tmp/cucr_office_src/obj

OMNI_KIT_ACCEPT_EULA=YES HUNAV_OFFICE_OBJ_DIR=/tmp/cucr_office_src/obj \
  ~/isaacsim/python.sh tools/isaac_convert_office.py
```

Outputs:

- `src/worlds/assets/office/*.usd` + `textures/`
- `src/worlds/office.usd` (composed stage)
- `src/maps/office.png` + `office.yaml`

Launcher: double-click `~/Desktop/Run Office Behaviors.desktop` **once**
(`--world office --config office_behaviors`). Do not start a second Isaac while
the first is coming up.

GUI: operator-confirmed 2026-08-13 (Validated **#57**).

Shared checklist for the next CUCR world (`house_museum`, …):
`isaac-social-nav/docs/CUCR_WORLD_PORT.md`.

## Lessons that bit this port (also in that checklist)

- Keep SDF `<mesh><scale>` (`0.01` rooms, **`0.001` desks**). Same class as hospital **#41**.
- No ceiling slab (blocks top-down).
- `UsdGeom.Cube` has no UVs — do not connect `UsdUVTexture` (black cave).
- Sidecar `textures/*.png` must be **absolute** on asset USDs; relative paths resolve
  against `worlds/office.usd`.
- Do not wire color maps into `opacity` unless the PNG has alpha.
- `setup.py` must install `worlds/assets/office/` (not only `worlds/*.usd`).
  Isaac was opening the colcon share copy; furniture refs failed silently.
- Compose payload refs as absolute paths. `find_package_share_directory` prefers
  source `src/worlds` when present.
- DistantLight 5000 + DomeLight ~400. Live Kit does not pick up a rewritten USD —
  close Isaac and relaunch.
- Viewport lighting: **Default** rig (`world_builder.py` — office is not in
  `_STAGE_LIGHT_WORLDS`). Bookstore/residential use Stage.

## Compose orientation fixes (GUI-verified, one asset at a time)

Some Assimp OBJ exports are **Y-up** (mesh height on **+Y**) while the Isaac
stage is **Z-up**. Gazebo tolerated this; composed payloads need **`RotateX(90)`**
on the **`geometry`** xform only (bookstore pattern). Do **not** reconvert asset
USDs or run `_patch_office_asset_usd` for these — **`--compose-only`** rewrite
`office.usd` only.

| Model | Symptom | Fix | Instances | GUI sign-off |
|---|---|---|---|---|
| `office_table` | Table on its side | `RotateX(90)` on payload | all `office_table_*` | 2026-08-31 |
| `office_chair` | Chair on its side | `RotateX(90)` on payload | 59× `office_chair_*` | 2026-08-31 |
| `toilet` | Toilet on its side (`toilet_16`) | `RotateX(90)` on payload | 3× `toilet_*` | 2026-08-31 |
| `wastebasket` | Bin on its side (`wastebasket_17`) | `RotateX(90)` on payload | 6× `wastebasket_*` | 2026-08-31 |
| `office_cafe_table` | Cafe table fallen | `RotateX(90)` on payload | 7× `office_cafe_table_*` | 2026-08-31 |
| `tv_stand` | TV unit fallen (`tv_stand_679`) | `RotateX(90)` on payload | 2× `tv_stand_*` | 2026-08-31 |
| `office_couch` | Couch fallen (`office_couch_677`) | `RotateX(90)` on payload | 3× `office_couch_*` | 2026-08-31 |

**Workflow:** add model name to `_COMPOSE_ROTATE_X_MODELS` in
`isaac_convert_office.py` → close Isaac →

```bash
OMNI_KIT_ACCEPT_EULA=YES HUNAV_OFFICE_OBJ_DIR=/tmp/cucr_office_src/obj \
  ~/isaacsim/python.sh tools/isaac_convert_office.py --compose-only
```

→ relaunch office → confirm in GUI → tick the table above.

**Do not:** bulk `_patch_office_asset_usd`, full reconvert all assets, or add
models to the rotate set without a screenshot — that regressed the whole scene
(2026-08-31).

**Still open (not rotate-tested):** `desk_chair`, `hangout_chair` (OBJ `scale
0.01` — separate class from DAE furniture above).
