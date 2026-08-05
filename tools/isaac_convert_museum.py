#!/usr/bin/env python3
"""
Convert CUCR museum meshes with Isaac's omni.kit.asset_converter (correct await API).

Prerequisite OBJ exports (Assimp is fine as a Collada front-end):
  assimp export .../new_museum.dae /tmp/cucr_museum_src/obj/museum.obj
  assimp export .../floor.dae     /tmp/cucr_museum_src/obj/floor.obj
  # keep textures + .mtl beside the OBJs

Usage:
  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/isaac_convert_museum.py
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "width": 960,
        "height": 540,
        "headless": True,
        "renderer": "RaytracedLighting",
        "sync_loads": True,
    }
)

from omni.kit.asset_converter import AssetConverterContext, get_instance
from omni.kit.async_engine import run_coroutine
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


SRC_OBJ_DIR = os.environ.get("HUNAV_MUSEUM_OBJ_DIR", "/tmp/cucr_museum_src/obj")
OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src", "worlds")
)
ASSETS_DIR = os.path.join(OUT_DIR, "assets", "museum")
MUSEUM_Z_OFFSET = 0.5  # Gazebo museum model.sdf visual pose z=0.5
CONVERT_TIMEOUT_S = 180.0


def _make_context() -> AssetConverterContext:
    ctx = AssetConverterContext()
    ctx.ignore_materials = False
    ctx.ignore_animations = True
    ctx.ignore_camera = True
    ctx.ignore_light = True
    ctx.export_preview_surface = True
    ctx.use_meter_as_world_unit = True
    ctx.create_world_as_default_root_prim = True
    ctx.embed_textures = True
    ctx.keep_all_materials = True
    ctx.merge_all_meshes = False
    # Assimp OBJ is Y-up; Isaac / Gazebo museum are Z-up.
    ctx.convert_stage_up_z = True
    ctx.convert_stage_up_y = False
    return ctx


def convert_asset(src: str, dst: str) -> None:
    if not os.path.isfile(src):
        raise FileNotFoundError(src)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    if os.path.exists(dst):
        os.remove(dst)

    mgr = get_instance()
    if mgr is None:
        raise RuntimeError("omni.kit.asset_converter get_instance() returned None")

    ctx = _make_context()

    async def _run():
        print(f"[convert] start {src} -> {dst}", flush=True)

        def progress(cur, total=None):
            if total:
                print(f"[convert] progress {cur}/{total}", flush=True)

        task = mgr.create_converter_task(src, dst, progress, ctx)
        # CRITICAL: must await wait_until_finished(); polling is_finished() alone never completes.
        ok = await task.wait_until_finished()
        status = task.get_status()
        err = task.get_error_message()
        print(f"[convert] done ok={ok} status={status} err={err!r}", flush=True)
        return ok, status, err

    fut = run_coroutine(_run())
    t0 = time.time()
    while not fut.done():
        simulation_app.update()
        if time.time() - t0 > CONVERT_TIMEOUT_S:
            raise TimeoutError(f"Convert timed out after {CONVERT_TIMEOUT_S}s: {src}")

    ok, status, err = fut.result()
    if not ok or not os.path.isfile(dst):
        raise RuntimeError(f"Convert failed {src} -> {dst}: status={status} err={err!r}")
    print(f"[convert] OK {dst} ({os.path.getsize(dst)} bytes)", flush=True)


def _mark_static_colliders(root_prim) -> None:
    UsdPhysics.RigidBodyAPI.Apply(root_prim)
    UsdPhysics.RigidBodyAPI(root_prim).CreateRigidBodyEnabledAttr(False)
    UsdPhysics.CollisionAPI.Apply(root_prim)
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(
                "meshSimplification"
            )
            PhysxSchema.PhysxCollisionAPI.Apply(prim)


def _add_payload(stage, path: str, rel_usd: str, translate: Gf.Vec3d) -> None:
    """Wrapper Xform owns T/R; child holds the Isaac-converted reference.

    Isaac OBJ import keeps Y-up mesh data even when stage up-axis is Z.
    Rotate +90° about X so height becomes +Z (Isaac / Gazebo convention).
    """
    xf = UsdGeom.Xform.Define(stage, path)
    # xformOpOrder applies right-to-left on points: rotate Y-up→Z-up, then translate.
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    tr.Set(translate)
    rot = xf.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble)
    rot.Set(90.0)

    child = UsdGeom.Xform.Define(stage, f"{path}/geometry")
    child.GetPrim().GetReferences().AddReference(rel_usd)


def compose_museum_stage(museum_usd: str, floor_usd: str, out_usd: str) -> None:
    if os.path.exists(out_usd):
        os.remove(out_usd)

    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    light = stage.DefinePrim("/World/DistantLight", "DistantLight")
    light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

    # Gazebo poses were in Z-up; apply them after Y-up→Z-up rotate.
    _add_payload(
        stage,
        "/World/museum_floor",
        "./assets/museum/museum_floor.usd",
        Gf.Vec3d(-0.001425, -0.014447, 0.0),
    )
    _add_payload(
        stage,
        "/World/museum",
        "./assets/museum/museum_mesh.usd",
        Gf.Vec3d(0.0, 0.0, MUSEUM_Z_OFFSET),
    )

    stage.GetRootLayer().Save()

    # Re-open so references resolve, then author static colliders.
    stage = Usd.Stage.Open(out_usd)
    for path in ("/World/museum_floor/geometry", "/World/museum/geometry"):
        root = stage.GetPrimAtPath(path)
        if root and root.IsValid():
            _mark_static_colliders(root)

    ground = UsdGeom.Mesh.Define(stage, "/World/GroundPlane")
    size = 120.0
    ground.CreatePointsAttr(
        [
            (-size, -size, -0.05),
            (size, -size, -0.05),
            (size, size, -0.05),
            (-size, size, -0.05),
        ]
    )
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateSubdivisionSchemeAttr().Set("none")
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(ground.GetPrim()).CreateApproximationAttr().Set(
        "none"
    )

    stage.GetRootLayer().Save()
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), ["default", "render"]
    ).ComputeWorldBound(stage.GetPrimAtPath("/World"))
    print(f"[compose] wrote {out_usd}", flush=True)
    print(f"[compose] bbox={bbox.GetRange()}", flush=True)
    print(f"[compose] prims={len(list(stage.Traverse()))}", flush=True)


def main() -> int:
    os.makedirs(ASSETS_DIR, exist_ok=True)
    museum_src = os.path.join(SRC_OBJ_DIR, "museum.obj")
    floor_src = os.path.join(SRC_OBJ_DIR, "floor.obj")
    museum_out = os.path.join(ASSETS_DIR, "museum_mesh.usd")
    floor_out = os.path.join(ASSETS_DIR, "museum_floor.usd")
    final_out = os.path.join(OUT_DIR, "museum.usd")

    convert_asset(museum_src, museum_out)
    convert_asset(floor_src, floor_out)
    compose_museum_stage(museum_out, floor_out, final_out)
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main()
    except Exception as exc:
        print(f"[FATAL] {exc}", flush=True)
        raise
    finally:
        simulation_app.close()
    sys.exit(rc)
