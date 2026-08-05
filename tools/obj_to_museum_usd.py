#!/usr/bin/env python3
"""Build museum.usd from Assimp OBJ exports without omni.kit.asset_converter."""

import os
import sys
from collections import defaultdict

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "width": 640,
        "height": 360,
        "headless": True,
        "renderer": "RaytracedLighting",
        "sync_loads": True,
    }
)

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt


SRC_OBJ_DIR = "/tmp/cucr_museum_src/obj"
OUT_DIR = (
    "/home/osamuzahid/Projects/isaac-social-nav/ros2_ws/src/"
    "Hunav_isaac_wrapper/src/worlds"
)
ASSETS_DIR = os.path.join(OUT_DIR, "assets", "museum")
MUSEUM_Z_OFFSET = 0.5


def parse_obj(path: str):
    """Return (points, meshes) where meshes is {name: (counts, indices)}."""
    positions = []
    # face entries as list of (mesh_name, [(v_idx,...), ...]) using 0-based v indices
    current = "mesh0"
    faces_by_mesh = defaultdict(list)

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts:
                continue
            tag = parts[0]
            if tag == "v" and len(parts) >= 4:
                # Assimp OBJ is Y-up; Isaac / Gazebo museum are Z-up.
                # Map (x, y_up, z) -> (x, z, y_up).
                x, y_up, z = float(parts[1]), float(parts[2]), float(parts[3])
                positions.append((x, z, y_up))
            elif tag == "o" or tag == "g":
                current = parts[1] if len(parts) > 1 else current
            elif tag == "usemtl" and len(parts) > 1:
                current = parts[1]
            elif tag == "f" and len(parts) >= 4:
                idxs = []
                for p in parts[1:]:
                    # formats: v, v/vt, v/vt/vn, v//vn
                    v = p.split("/")[0]
                    idxs.append(int(v) - 1)
                # triangulate fan
                for i in range(1, len(idxs) - 1):
                    faces_by_mesh[current].append((idxs[0], idxs[i], idxs[i + 1]))

    meshes = {}
    for name, tris in faces_by_mesh.items():
        counts = [3] * len(tris)
        indices = [i for tri in tris for i in tri]
        meshes[name] = (counts, indices)
    return positions, meshes


def write_mesh_usd(obj_path: str, usd_path: str, root_name: str) -> None:
    positions, meshes = parse_obj(obj_path)
    if not positions or not meshes:
        raise RuntimeError(f"No geometry in {obj_path}")

    if os.path.exists(usd_path):
        os.remove(usd_path)

    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}")
    stage.SetDefaultPrim(root.GetPrim())

    # Static rigid body on root so Physics sees the hierarchy
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.RigidBodyAPI(root.GetPrim()).CreateRigidBodyEnabledAttr(False)

    points = Vt.Vec3fArray([Gf.Vec3f(*p) for p in positions])
    for i, (name, (counts, indices)) in enumerate(meshes.items()):
        safe = "".join(c if c.isalnum() or c in "_-" else "_" for c in name) or f"m{i}"
        mesh = UsdGeom.Mesh.Define(stage, f"/{root_name}/{safe}")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr().Set("none")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(
            "meshSimplification"
        )

    stage.GetRootLayer().Save()
    n_tris = sum(len(v[1]) // 3 for v in meshes.values())
    print(
        f"[mesh] {usd_path}: verts={len(positions)} groups={len(meshes)} tris={n_tris}",
        flush=True,
    )


def compose(museum_usd: str, floor_usd: str, out_usd: str) -> None:
    if os.path.exists(out_usd):
        os.remove(out_usd)
    stage = Usd.Stage.CreateNew(out_usd)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    light = stage.DefinePrim("/World/DistantLight", "DistantLight")
    light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(3000.0)

    floor_xf = UsdGeom.Xform.Define(stage, "/World/museum_floor")
    # Relative to museum.usd in worlds/ so colcon share install still resolves.
    floor_xf.GetPrim().GetReferences().AddReference("./assets/museum/museum_floor.usd")
    floor_xf.AddTranslateOp().Set(Gf.Vec3d(-0.001425, -0.014447, 0.0))

    museum_xf = UsdGeom.Xform.Define(stage, "/World/museum")
    museum_xf.GetPrim().GetReferences().AddReference("./assets/museum/museum_mesh.usd")
    museum_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, MUSEUM_Z_OFFSET))

    # Thin ground under everything
    ground = UsdGeom.Mesh.Define(stage, "/World/GroundPlane")
    size = 120.0
    ground.CreatePointsAttr(
        [(-size, -size, -0.05), (size, -size, -0.05), (size, size, -0.05), (-size, size, -0.05)]
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
    ).ComputeWorldBound(world.GetPrim())
    print(f"[compose] {out_usd}", flush=True)
    print(f"[compose] bbox={bbox.GetRange()}", flush=True)
    print(f"[compose] prims={len(list(stage.Traverse()))}", flush=True)


def main() -> int:
    os.makedirs(ASSETS_DIR, exist_ok=True)
    museum_mesh = os.path.join(ASSETS_DIR, "museum_mesh.usd")
    floor_mesh = os.path.join(ASSETS_DIR, "museum_floor.usd")
    final_usd = os.path.join(OUT_DIR, "museum.usd")

    write_mesh_usd(os.path.join(SRC_OBJ_DIR, "museum.obj"), museum_mesh, "museum_mesh")
    write_mesh_usd(os.path.join(SRC_OBJ_DIR, "floor.obj"), floor_mesh, "museum_floor")
    compose(museum_mesh, floor_mesh, final_usd)
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    sys.exit(rc)
