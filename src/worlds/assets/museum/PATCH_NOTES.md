# Museum asset patches (isaac-social-nav)

These USD files are produced by `tools/isaac_convert_museum.py` from CUCR
`cucr_worlds_museum`. Some look/feel values are **intentionally different**
from the Gazebo source.

| What | Originally (CUCR / faithful import) | Patch (this project) |
|---|---|---|
| Floor diffuse | Neon blue `Kd ≈ (0, 0, 0.8)` in `floor.dae` / `floor.mtl` | Light warm brown `(0.62, 0.48, 0.34)` to sit with dark brown walls |
| GroundPlane | Bare mesh (no material) → default white from above | Same light-brown PreviewSurface (`GroundBrown`) |
| Distant light | Intensity `3000` in first compose | Intensity `5000` for laptop visibility |
| Up-axis | Assimp/Isaac OBJ mesh data is Y-up | Compose wrapper `RotateX +90` to Z-up |

Search `ORIGINALLY` / `PATCH (isaac-social-nav)` in `tools/isaac_convert_museum.py`.
Re-run that script after changing source meshes so patches are reapplied.

## Props / art

CUCR Gazebo `museum.world` only includes `museum` + `museum_floor` (no separate
art/bench model includes). Benches/art visible in Isaac come from geometry
already baked into `new_museum.dae` → `museum_mesh.usd`. There is no deferred
museum prop port beyond optional `museum_sealing` (not referenced by the world).
