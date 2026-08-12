"""
lab_robot_sensors.py

PATCH (isaac-social-nav): attach stock critical sensors for CUCR lab robots and
publish TF / joint_states / lidar / IMU / RGB-D via isaacsim.ros2.bridge.

URDF→USD gives morphology only. Lidar/cameras/IMU are Isaac sensor prims parented
under the correct frames (see docs/ROBOTS.md in isaac-social-nav).

Env:
  HUNAV_LAB_SENSORS=0|1   — master switch (default 1 for lab robots)
  HUNAV_LAB_CAMERAS=0|1   — RGB-D (default 0; heavy on under-spec laptops)
  HUNAV_LAB_LIDAR=0|1     — RTX 2D lidar (default 1 for Stretch)
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
import omni.graph.core as og

# Stretch SE3 link paths relative to robot root (`/World/Stretch`).
_STRETCH_BASE = "Geometry/base_link"
_STRETCH_LASER = f"{_STRETCH_BASE}/laser"
_STRETCH_IMU = f"{_STRETCH_BASE}/base_imu"
# ROS optical frame: +Z is the RealSense look axis (+Y down). On parked Stretch,
# camera_link +X is NOT that look axis (head_tilt joint origin is +90° X) — mounting
# there looked straight down at the base. Always parent under optical.
_STRETCH_CAMERA_LINK = (
    f"{_STRETCH_BASE}/link_mast/link_head/link_head_pan/link_head_tilt/"
    "camera_bottom_screw_frame/camera_link"
)
_STRETCH_CAMERA_OPTICAL = (
    f"{_STRETCH_CAMERA_LINK}/camera_color_frame/camera_color_optical_frame"
)


def _orient_opengl_camera_on_optical(stage, cam_prim, optical_path: str) -> None:
    """UsdGeom.Camera under ROS optical: look along +Z_opt, roll leveled to world +Z.

    OpenGL cameras look along local −Z with +Y up. Solve one local quaternion from
    world bases (no Euler patch stack). Look = optical +Z; up = world +Z projected
    orthogonal to look (so rqt is upright even when optical −Y is horizontal).
    """
    from pxr import Gf, Usd, UsdGeom

    opt = stage.GetPrimAtPath(optical_path)
    if not opt or not opt.IsValid():
        raise RuntimeError(f"missing optical prim {optical_path}")
    mw = UsdGeom.Xformable(opt).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    forward = Gf.Vec3d(mw.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)))
    if forward.GetLength() < 1e-8:
        raise RuntimeError(f"optical +Z degenerate at {optical_path}")
    forward.Normalize()

    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    right = Gf.Cross(forward, world_up)
    if right.GetLength() < 1e-3:
        # Nearly vertical look: fall back to optical −Y as the up hint.
        hint = Gf.Vec3d(mw.TransformDir(Gf.Vec3d(0.0, -1.0, 0.0)))
        right = Gf.Cross(forward, hint)
    right.Normalize()
    up = Gf.Cross(right, forward)
    up.Normalize()

    # Row-major: columns are camera axes in world (X=right, Y=up, Z=−look).
    r_cam = Gf.Matrix3d(
        right[0],
        up[0],
        -forward[0],
        right[1],
        up[1],
        -forward[1],
        right[2],
        up[2],
        -forward[2],
    )
    r_local = mw.ExtractRotationMatrix().GetInverse() * r_cam
    quat = Gf.Quatf(r_local.ExtractRotation().GetQuat())

    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(quat)

# Parked Stretch joint names (kinematic Physics=none has no articulation).
_STRETCH_PARKED_JOINTS = [
    "joint_left_wheel",
    "joint_right_wheel",
    "joint_lift",
    "joint_arm_l0",
    "joint_arm_l1",
    "joint_arm_l2",
    "joint_arm_l3",
    "joint_arm_l4",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
    "joint_head_pan",
    "joint_head_tilt",
    "joint_gripper_finger_left",
    "joint_gripper_finger_right",
]


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def lab_sensors_enabled(robot_name: str) -> bool:
    if robot_name not in ("franka", "stretch", "stretch_wheeled"):
        return False
    return _env_bool("HUNAV_LAB_SENSORS", True)


def _prim_exists(path: str) -> bool:
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(path)
        return bool(prim and prim.IsValid())
    except Exception:
        return False


def _join(root: str, rel: str) -> str:
    return f"{root.rstrip('/')}/{rel.lstrip('/')}"


def _read_laser_scan_metadata(prim) -> Dict[str, Any]:
    rotation_rate = float(prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get() or 0)
    near_range = float(prim.GetAttribute("omni:sensor:Core:nearRangeM").Get() or 0)
    far_range = float(prim.GetAttribute("omni:sensor:Core:farRangeM").Get() or 0)
    firing_rate = int(prim.GetAttribute("omni:sensor:Core:patternFiringRateHz").Get() or 0)
    if rotation_rate <= 0 or firing_rate <= 0:
        # Safe defaults matching Example_Rotary_2D-ish behavior.
        return {
            "horizontalFov": 360.0,
            "horizontalResolution": 1.0,
            "depthRange": [0.1, 30.0],
            "rotationRate": 10.0,
            "azimuthRange": [-180.0, 180.0],
        }
    return {
        "horizontalFov": 360.0,
        "horizontalResolution": 360.0 * rotation_rate / firing_rate,
        "depthRange": [near_range, far_range],
        "rotationRate": rotation_rate,
        "azimuthRange": [-180.0, 180.0],
    }


def _attach_tf_tree(graph_path: str, target_prims: Sequence[str], topic: str = "tf") -> None:
    import usdrt.Sdf

    keys = og.Controller.Keys
    paths = [usdrt.Sdf.Path(p) for p in target_prims if _prim_exists(p)]
    if not paths:
        print(f"[lab_robot_sensors] TF skip: no valid target prims for {graph_path}")
        return
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishTF.inputs:timeStamp",
                    ),
                ],
                keys.SET_VALUES: [
                    ("PublishTF.inputs:topicName", topic),
                    ("PublishTF.inputs:targetPrims", paths),
                ],
            },
        )
        print(f"[lab_robot_sensors] TF graph {graph_path} → /{topic} ({len(paths)} prims)")
    except Exception as exc:
        print(f"[lab_robot_sensors] TF graph failed: {exc}")


def _attach_joint_state_publisher(
    graph_path: str, articulation_prim: str, topic: str = "joint_states"
) -> bool:
    import usdrt.Sdf

    if not _prim_exists(articulation_prim):
        print(
            f"[lab_robot_sensors] joint_states skip: missing prim {articulation_prim}"
        )
        return False
    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                    (
                        "ReadJointState.outputs:execOut",
                        "PublishJointState.inputs:execIn",
                    ),
                    (
                        "ReadJointState.outputs:jointNames",
                        "PublishJointState.inputs:jointNames",
                    ),
                    (
                        "ReadJointState.outputs:jointPositions",
                        "PublishJointState.inputs:jointPositions",
                    ),
                    (
                        "ReadJointState.outputs:jointVelocities",
                        "PublishJointState.inputs:jointVelocities",
                    ),
                    (
                        "ReadJointState.outputs:jointEfforts",
                        "PublishJointState.inputs:jointEfforts",
                    ),
                    (
                        "ReadJointState.outputs:jointDofTypes",
                        "PublishJointState.inputs:jointDofTypes",
                    ),
                    (
                        "ReadJointState.outputs:stageMetersPerUnit",
                        "PublishJointState.inputs:stageMetersPerUnit",
                    ),
                    (
                        "ReadJointState.outputs:sensorTime",
                        "PublishJointState.inputs:sensorTime",
                    ),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                ],
                keys.SET_VALUES: [
                    (
                        "ReadJointState.inputs:prim",
                        [usdrt.Sdf.Path(articulation_prim)],
                    ),
                    ("PublishJointState.inputs:topicName", topic),
                    ("Context.inputs:useDomainIDEnvVar", True),
                ],
            },
        )
        print(
            f"[lab_robot_sensors] joint_states graph {graph_path} "
            f"prim={articulation_prim} → /{topic}"
        )
        return True
    except Exception as exc:
        print(f"[lab_robot_sensors] joint_states graph failed: {exc}")
        return False


def _level_rtx_2d_lidar_beam(lidar_prim) -> None:
    """Set Example_Rotary_2D emitter elevation to 0° (horizontal).

    NVIDIA's stock ``Example_Rotary_2D`` profile aims rays at elevationDeg=[-2].
    A real Stretch SE3 RPLidar scans in the horizontal plane of the ``laser``
    frame. Override the OmniLidar emitter-state attribute — do not pitch the
    lidar prim to cancel the example tilt.
    """
    changed = []
    for attr in lidar_prim.GetAttributes():
        name = attr.GetName()
        if "emitterState" not in name or not name.endswith(":elevationDeg"):
            continue
        prev = attr.Get()
        n = len(list(prev)) if prev is not None else 1
        attr.Set([0.0] * n)
        changed.append(f"{name} (was {list(prev) if prev is not None else None})")
    if changed:
        print(
            "[lab_robot_sensors] lidar beam leveled to 0° (RPLidar-horizontal): "
            + "; ".join(changed)
        )
    else:
        print(
            "[lab_robot_sensors] WARNING: no emitterState:*/elevationDeg on lidar "
            "prim — stock −2° tilt may remain"
        )


def _attach_stretch_lidar(laser_prim: str) -> Optional[Any]:
    if not _env_bool("HUNAV_LAB_LIDAR", True):
        print("[lab_robot_sensors] lidar disabled (HUNAV_LAB_LIDAR=0)")
        return None
    if not _prim_exists(laser_prim):
        print(f"[lab_robot_sensors] lidar skip: missing {laser_prim}")
        return None
    try:
        import isaacsim.core.experimental.utils.prim as prim_utils
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

        lidar_path = f"{laser_prim}/rtx_lidar"
        # Base NVIDIA 2D rotary asset at the URDF ``laser`` mount. Elevation is
        # corrected after create (see ``_level_rtx_2d_lidar_beam``).
        lidar = Lidar.create(
            path=lidar_path,
            config="Example_Rotary_2D",
            tick_rate=10.0,
            translations=[[0.0, 0.0, 0.0]],
        )
        _level_rtx_2d_lidar_beam(lidar.prims[0])
        # NVIDIA requires tickRate == scanRateBaseHz for a full rotary scan per
        # published LaserScan (stock Example_Rotary_2D is 30 Hz).
        prim = lidar.prims[0]
        scan_hz = prim.GetAttribute("omni:sensor:Core:scanRateBaseHz").Get()
        if scan_hz is not None and float(scan_hz) > 0:
            tick_attr = prim.GetAttribute("omni:sensor:tickRate")
            if tick_attr and tick_attr.IsValid():
                prev = tick_attr.Get()
                tick_attr.Set(float(scan_hz))
                print(
                    f"[lab_robot_sensors] lidar tickRate {prev} → {scan_hz} "
                    "(match scanRateBaseHz)"
                )
        sensor = LidarSensor(lidar, annotators=[])
        meta = _read_laser_scan_metadata(prim_utils.get_prim_at_path(lidar.paths[0]))
        sensor.attach_writer(
            "RtxLidarROS2PublishLaserScan",
            topicName="scan",
            frameId="laser",
            **meta,
        )
        print(f"[lab_robot_sensors] RTX 2D lidar at {lidar.paths[0]} → /scan")
        return sensor
    except Exception as exc:
        print(f"[lab_robot_sensors] lidar attach failed: {exc}")
        return None


def _attach_stretch_camera(optical_prim: str, width: int = 320, height: int = 240) -> bool:
    if not _env_bool("HUNAV_LAB_CAMERAS", False):
        print("[lab_robot_sensors] cameras off (set HUNAV_LAB_CAMERAS=1 to enable)")
        return False
    if not _prim_exists(optical_prim):
        print(f"[lab_robot_sensors] camera skip: missing {optical_prim}")
        return False
    try:
        import omni.usd
        import usdrt.Sdf
        from pxr import Sdf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        # Drop stale mounts (camera_link and prior optical).
        link_prim = optical_prim.rsplit("/camera_color_frame", 1)[0]
        for stale in (
            f"{link_prim}/rgb_camera",
            f"{optical_prim}/rgb_camera",
        ):
            if stage.GetPrimAtPath(stale):
                stage.RemovePrim(stale)

        cam_path = f"{optical_prim}/rgb_camera"
        cam = UsdGeom.Camera(stage.DefinePrim(cam_path, "Camera"))
        _orient_opengl_camera_on_optical(stage, cam.GetPrim(), optical_prim)
        cam.GetHorizontalApertureAttr().Set(21)
        cam.GetVerticalApertureAttr().Set(16)
        cam.GetProjectionAttr().Set("perspective")
        cam.GetFocalLengthAttr().Set(24)
        cam.GetPrim().CreateAttribute("exposure:time", Sdf.ValueTypeNames.Float).Set(0.02)

        keys = og.Controller.Keys
        og.Controller.edit(
            {
                "graph_path": "/World/ROS2_LabCamera",
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "createRenderProduct",
                        "isaacsim.core.nodes.IsaacCreateRenderProduct",
                    ),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    (
                        "OnPlaybackTick.outputs:tick",
                        "createRenderProduct.inputs:execIn",
                    ),
                    (
                        "createRenderProduct.outputs:execOut",
                        "cameraHelperRgb.inputs:execIn",
                    ),
                    (
                        "createRenderProduct.outputs:execOut",
                        "cameraHelperInfo.inputs:execIn",
                    ),
                    (
                        "createRenderProduct.outputs:execOut",
                        "cameraHelperDepth.inputs:execIn",
                    ),
                    (
                        "createRenderProduct.outputs:renderProductPath",
                        "cameraHelperRgb.inputs:renderProductPath",
                    ),
                    (
                        "createRenderProduct.outputs:renderProductPath",
                        "cameraHelperInfo.inputs:renderProductPath",
                    ),
                    (
                        "createRenderProduct.outputs:renderProductPath",
                        "cameraHelperDepth.inputs:renderProductPath",
                    ),
                ],
                keys.SET_VALUES: [
                    (
                        "createRenderProduct.inputs:cameraPrim",
                        [usdrt.Sdf.Path(cam_path)],
                    ),
                    ("createRenderProduct.inputs:width", width),
                    ("createRenderProduct.inputs:height", height),
                    ("cameraHelperRgb.inputs:frameId", "camera_color_optical_frame"),
                    ("cameraHelperRgb.inputs:topicName", "camera/color/image_raw"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", "camera_color_optical_frame"),
                    ("cameraHelperInfo.inputs:topicName", "camera/color/camera_info"),
                    ("cameraHelperDepth.inputs:frameId", "camera_color_optical_frame"),
                    (
                        "cameraHelperDepth.inputs:topicName",
                        "camera/depth/image_rect_raw",
                    ),
                    ("cameraHelperDepth.inputs:type", "depth"),
                ],
            },
        )
        print(
            f"[lab_robot_sensors] RGB-D camera at {cam_path} "
            f"({width}x{height}) → /camera/color/* /camera/depth/* "
            "(optical +Z look, world-up roll)"
        )
        return True
    except Exception as exc:
        print(f"[lab_robot_sensors] camera attach failed: {exc}")
        return False


def _attach_synthetic_imu_publisher(
    graph_path: str, frame_id: str = "base_imu", topic: str = "imu"
) -> None:
    """Publish gravity-only IMU for static / kinematic bases (no PhysX body needed)."""
    keys = og.Controller.Keys
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishImu.inputs:execIn"),
                    ("Context.outputs:context", "PublishImu.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishImu.inputs:timeStamp",
                    ),
                ],
                keys.SET_VALUES: [
                    ("PublishImu.inputs:topicName", topic),
                    ("PublishImu.inputs:frameId", frame_id),
                    ("PublishImu.inputs:linearAcceleration", [0.0, 0.0, 9.81]),
                    ("PublishImu.inputs:angularVelocity", [0.0, 0.0, 0.0]),
                    ("PublishImu.inputs:orientation", [0.0, 0.0, 0.0, 1.0]),
                    ("Context.inputs:useDomainIDEnvVar", True),
                ],
            },
        )
        print(
            f"[lab_robot_sensors] synthetic IMU → /{topic} "
            f"(frame={frame_id}, gravity-only)"
        )
    except Exception as exc:
        print(f"[lab_robot_sensors] IMU publish failed: {exc}")


class ParkedJointStatePublisher:
    """rclpy publisher of fixed joint_states for kinematic Stretch (Physics=none)."""

    def __init__(self, node, joint_names: Optional[Sequence[str]] = None):
        from sensor_msgs.msg import JointState

        self._JointState = JointState
        self._names = list(joint_names or _STRETCH_PARKED_JOINTS)
        self._pub = node.create_publisher(JointState, "/joint_states", 10)
        self._zeros = [0.0] * len(self._names)

    def publish(self, stamp_sec: float = 0.0) -> None:
        msg = self._JointState()
        msg.header.stamp.sec = int(stamp_sec)
        msg.header.stamp.nanosec = int((stamp_sec % 1.0) * 1e9)
        msg.name = list(self._names)
        msg.position = list(self._zeros)
        msg.velocity = list(self._zeros)
        msg.effort = list(self._zeros)
        self._pub.publish(msg)


def attach_lab_robot_sensors(
    robot_name: str,
    robot_prim_path: str,
    ros_node=None,
) -> Dict[str, Any]:
    """
    Attach stock sensors for a lab robot. Returns a handle dict (may include
    lidar sensor objects and optional ParkedJointStatePublisher).
    """
    handles: Dict[str, Any] = {"robot": robot_name, "prim": robot_prim_path}
    if not lab_sensors_enabled(robot_name):
        print(f"[lab_robot_sensors] skipped for {robot_name}")
        return handles

    if robot_name == "franka":
        _attach_tf_tree("/World/ROS2_LabTF", [robot_prim_path])
        _attach_joint_state_publisher("/World/ROS2_LabJoints", robot_prim_path)
        # Wrist F/T: optional later if lab cell includes it (ROBOTS.md open Q).
        print("[lab_robot_sensors] Franka: TF + joint_states attached")
        return handles

    if robot_name in ("stretch", "stretch_wheeled"):
        base = _join(robot_prim_path, _STRETCH_BASE)
        laser = _join(robot_prim_path, _STRETCH_LASER)
        imu = _join(robot_prim_path, _STRETCH_IMU)
        cam_link = _join(robot_prim_path, _STRETCH_CAMERA_LINK)
        cam_optical = _join(robot_prim_path, _STRETCH_CAMERA_OPTICAL)

        tf_targets = [
            p
            for p in (robot_prim_path, base, laser, imu, cam_link)
            if _prim_exists(p)
        ]
        # Prefer articulation root for wheeled so full link tree publishes.
        _attach_tf_tree("/World/ROS2_LabTF", tf_targets or [robot_prim_path])

        if robot_name == "stretch_wheeled":
            ok = _attach_joint_state_publisher("/World/ROS2_LabJoints", robot_prim_path)
            if not ok and ros_node is not None:
                handles["parked_js"] = ParkedJointStatePublisher(ros_node)
        elif ros_node is not None:
            handles["parked_js"] = ParkedJointStatePublisher(ros_node)
            print(
                "[lab_robot_sensors] Stretch kinematic: publishing parked "
                "/joint_states via rclpy (Physics=none)"
            )

        handles["lidar"] = _attach_stretch_lidar(laser)
        _attach_synthetic_imu_publisher("/World/ROS2_LabImu", frame_id="base_imu")
        _attach_stretch_camera(cam_optical)
        print("[lab_robot_sensors] Stretch: TF + joints + lidar + IMU (+ optional RGB-D)")
        return handles

    return handles


def tick_lab_sensor_handles(handles: Optional[Dict[str, Any]], sim_time: float = 0.0) -> None:
    if not handles:
        return
    parked = handles.get("parked_js")
    if parked is not None:
        parked.publish(sim_time)
