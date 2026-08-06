#!/usr/bin/env python3
"""
teleop_hunav_sim.py

Contains the TeleopHuNavSim class which combines:
- ROS 2 teleoperation for a differential robot.
- World loading via WorldBuilder.
- Agent management via HuNavManager.
"""
from isaacsim import SimulationApp

# ---------------------------------------------------------------------------
# ORIGINALLY (upstream v2.0): hard-coded SimulationApp CONFIG — kept for reference.
# Had to be patched because this laptop is under Isaac 6.0 minima (8GB VRAM / ~16GB
# RAM): always launching 1280x720 windowed RaytracedLighting OOMs / thrashes swap.
# ---------------------------------------------------------------------------
# CONFIG = {
#     "width": 1280,
#     "height": 720,
#     "sync_loads": True,
#     "headless": False,
#     "renderer": "RaytracedLighting",
# }
# simulation_app = SimulationApp(CONFIG)
# ---------------------------------------------------------------------------
# PATCH (isaac-social-nav): selectable profiles via sim_app_config.py
# (HUNAV_ISAAC_PROFILE / --profile / --debug). default|lab keeps the original
# 1280x720 windowed settings; debug|laptop uses 960x540 headless so bring-up
# and E2E smoke can run on this host. Also injects Kit --enable for anim graph.
# ---------------------------------------------------------------------------
from .sim_app_config import build_simulation_config, simulation_app_kwargs

_CONFIG_FULL = build_simulation_config()
CONFIG = simulation_app_kwargs(_CONFIG_FULL)
print(
    f"[hunav_isaac_wrapper] SimulationApp profile={_CONFIG_FULL.get('_profile')} "
    f"{CONFIG.get('width')}x{CONFIG.get('height')} headless={CONFIG.get('headless')} "
    f"renderer={CONFIG.get('renderer')}"
)
simulation_app = SimulationApp(CONFIG)

import os
import signal
import subprocess
import math
import yaml
import numpy as np
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import Twist
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
import omni
import omni.graph.core as og
 
# Import the WorldBuilder and HuNavManager modules.
from .world_builder import WorldBuilder
from .hunav_manager import HuNavManager

def find_package_share_directory():
    """
    Find the package share directory containing worlds, scenarios, config, etc.
    Works both in development and installed package modes.
    """
    # Try to find via ROS2 package first (installed mode)
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "hunav_isaac_wrapper"],
            capture_output=True, text=True, check=True
        )
        pkg_path = Path(result.stdout.strip())
        share_dir = pkg_path / "share" / "hunav_isaac_wrapper"
        if share_dir.exists() and (share_dir / "worlds").exists():
            return str(share_dir)
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    # Development mode fallback
    current_file = Path(__file__)
    
    # Check if we're in src/hunav_isaac_wrapper/ (development mode)
    if current_file.parent.parent.name == "src":
        src_dir = current_file.parent.parent
        if (src_dir / "worlds").exists():
            return str(src_dir)
    
    # Last fallback - check current working directory
    cwd = Path.cwd()
    if (cwd / "worlds").exists():
        return str(cwd)
    
    # If all else fails, return the old path calculation
    return os.path.dirname(os.path.dirname(__file__))


def find_robot_config_path(filename):
    """
    Find robot configuration file in development or installed package.
    
    Args:
        filename: Name of the robot config file (e.g., "nova_carter_ros2_sensors.usd")
    
    Returns:
        str: Absolute path to the robot config file
    """
    # Try to find via ROS2 package share directory (installed mode)
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "hunav_isaac_wrapper"],
            capture_output=True, text=True, check=True
        )
        pkg_path = Path(result.stdout.strip())
        robot_config = pkg_path / "share" / "hunav_isaac_wrapper" / "config" / "robots" / filename
        if robot_config.exists():
            return str(robot_config)
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    
    # Try development mode (relative to this file)
    current_file_dir = Path(__file__).parent
    workspace_root = current_file_dir.parent.parent
    robot_config = workspace_root / "config" / "robots" / filename
    if robot_config.exists():
        return str(robot_config)
    
    # Try alternative development paths
    dev_paths = [
        current_file_dir.parent / "config" / "robots" / filename,
        Path.cwd() / "src" / "config" / "robots" / filename,
        Path.cwd() / "config" / "robots" / filename,
    ]
    
    for path in dev_paths:
        if path.exists():
            return str(path)
    
    raise FileNotFoundError(f"Robot config file not found: {filename}")

    raise FileNotFoundError(f"Robot config file not found: {filename}")


def _load_robot_init_pose(hunav_config_path):
    """
    PATCH (isaac-social-nav): optional robot_init_pose from hunav scenario YAML.
    Returns dict with x,y,z,h (defaults 0).
    """
    pose = {"x": 0.0, "y": 0.0, "z": 0.0, "h": 0.0}
    if not hunav_config_path:
        return pose
    config_path = Path(hunav_config_path)
    if not config_path.is_file():
        return pose
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        init = data.get("hunav_loader", {}).get("ros__parameters", {}).get(
            "robot_init_pose", {}
        )
        if isinstance(init, dict):
            pose.update({k: float(init.get(k, pose[k])) for k in pose})
    except Exception as exc:
        print(f"[hunav_isaac_wrapper] WARNING: could not read robot_init_pose: {exc}")
    return pose


def _yaw_to_orientation(h):
    """Quaternion (w, x, y, z) from yaw about +Z."""
    return [math.cos(h / 2.0), 0.0, 0.0, math.sin(h / 2.0)]


def _yaw_from_orientation(quat):
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ChassisDriveRobot:
    """
    PATCH (isaac-social-nav): kinematic planar base for Stretch.
    Spawns a vendored USD and integrates /cmd_vel on the root Xform each step.
    Arm / manipulator DOFs are not driven (visual-only articulation).
    """

    def __init__(self, prim_path, usd_path, position, orientation):
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        self._xform = SingleXFormPrim(
            prim_path=prim_path,
            name="Robot",
            position=position,
            orientation=orientation,
        )
        self._lin_vel = np.zeros(3, dtype=float)
        self._ang_vel = np.zeros(3, dtype=float)

    def apply_cmd_vel(self, lin_x, ang_z, dt):
        pos, quat = self._xform.get_world_pose()
        yaw = _yaw_from_orientation(quat)
        yaw += ang_z * dt
        pos = np.asarray(pos, dtype=float)
        pos[0] += lin_x * math.cos(yaw) * dt
        pos[1] += lin_x * math.sin(yaw) * dt
        new_quat = np.asarray(_yaw_to_orientation(yaw), dtype=float)
        self._xform.set_world_pose(position=pos, orientation=new_quat)
        self._lin_vel = np.array(
            [lin_x * math.cos(yaw), lin_x * math.sin(yaw), 0.0], dtype=float
        )
        self._ang_vel = np.array([0.0, 0.0, ang_z], dtype=float)

    def get_world_pose(self):
        return self._xform.get_world_pose()

    def get_linear_velocity(self):
        return self._lin_vel.copy()

    def get_angular_velocity(self):
        return self._ang_vel.copy()


class TeleopHuNavSim(Node):
    """
    Combines:
    - Differential robot teleop (subscribing to /cmd_vel)
    - USD map loading (via WorldBuilder)
    - Agent management and update (via HuNavManager)
    """

    def __init__(self, map_name, hunav_config, robot_name):
        super().__init__("hunav_sim")
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Assets root
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            print("Could not find Nucleus root.")

        # Load USD stage
        self.builder = WorldBuilder(base_path=find_package_share_directory())
        if map_name:
            self.builder.load_map(map_name)

        # Create World object
        timestep = 1.0 / 20.0
        self.world = World(
            stage_units_in_meters=1, physics_dt=timestep, rendering_dt=timestep
        )

        if map_name == "empty_world":
            self.world.scene.add_default_ground_plane()

        # ---------------------------------------------------------------------------
        # ORIGINALLY (upstream v2.0) — Isaac 4.5/5.x robot USD layout. Kept for reference.
        # Had to be patched because on Isaac 6.0 CDN these paths return HTTP 404
        # (robots moved under NVIDIA/ / iRobot/Create3/; Carter sensors USD gone;
        # carter_ROS zip not extracted by default).
        # ---------------------------------------------------------------------------
        # robot_configs = {
        #     "jetbot": {
        #         "name": "Jetbot",
        #         "usd_relative_path": os.path.join(
        #             "Isaac", "Robots", "Jetbot", "jetbot.usd"
        #         ),
        #         "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
        #         "wheel_radius": 0.0325,
        #         "wheel_base": 0.118,
        #     },
        #     "create3": {
        #         "name": "Create3",
        #         "usd_relative_path": os.path.join(
        #             "Isaac", "Robots", "iRobot", "create_3.usd"
        #         ),
        #         "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
        #         "wheel_radius": 0.03575,
        #         "wheel_base": 0.233,
        #     },
        #     "carter": {
        #         "name": "Nova_Carter",
        #         "usd_relative_path": os.path.join(
        #             "Isaac", "Robots", "Carter", "nova_carter_sensors.usd"
        #         ),
        #         "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
        #         "wheel_radius": 0.14,
        #         "wheel_base": 0.413,
        #     },
        #     "carter_ROS": {
        #         "name": "Nova_Carter",
        #         "usd_relative_path": find_robot_config_path(
        #             "nova_carter_ros2_sensors.usd"
        #         ),
        #         "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
        #         "wheel_radius": 0.14,
        #         "wheel_base": 0.413,
        #     },
        # }
        # ---------------------------------------------------------------------------
        # PATCH (isaac-social-nav): remap usd_relative_path to Isaac 6.0 CDN locations
        # verified HTTP 200 + omni.client.stat OK. Wheel DOFs / radii unchanged.
        # Important so robots actually spawn instead of failing asset resolve.
        # ---------------------------------------------------------------------------
        robot_configs = {
            "jetbot": {
                "name": "Jetbot",
                "usd_relative_path": os.path.join(
                    "Isaac", "Robots", "NVIDIA", "Jetbot", "jetbot.usd"
                ),
                "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
                "wheel_radius": 0.0325,
                "wheel_base": 0.118,
            },
            "create3": {
                "name": "Create3",
                "usd_relative_path": os.path.join(
                    "Isaac", "Robots", "iRobot", "Create3", "create_3.usd"
                ),
                "wheel_dof_names": ["left_wheel_joint", "right_wheel_joint"],
                "wheel_radius": 0.03575,
                "wheel_base": 0.233,
            },
            "carter": {
                "name": "Nova_Carter",
                "usd_relative_path": os.path.join(
                    "Isaac", "Robots", "NVIDIA", "NovaCarter", "nova_carter.usd"
                ),
                "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
                "wheel_radius": 0.14,
                "wheel_base": 0.413,
            },
            "carter_ROS": {
                "name": "Nova_Carter",
                "usd_relative_path": os.path.join(
                    "Isaac", "Samples", "ROS2", "Robots", "Nova_Carter_ROS.usd"
                ),
                "wheel_dof_names": ["joint_wheel_left", "joint_wheel_right"],
                "wheel_radius": 0.14,
                "wheel_base": 0.413,
            },
            # PATCH (isaac-social-nav): Hello Robot Stretch — vendored USD.
            # Two selectable drives:
            #   stretch           — kinematic chassis_only (no wall collision; arm visual-only)
            #   stretch_wheeled   — PhysX WheeledRobot + DifferentialController (collides)
            "stretch": {
                "name": "Stretch",
                "usd_relative_path": find_robot_config_path("stretch/stretch.usd"),
                "drive": "chassis_only",
            },
            "stretch_wheeled": {
                "name": "Stretch",
                "usd_relative_path": find_robot_config_path("stretch/stretch.usd"),
                "wheel_dof_names": ["joint_left_wheel", "joint_right_wheel"],
                # URDF: wheel centers at y=±0.17035, z=0.0508 → radius 0.0508, base ~0.3407
                "wheel_radius": 0.0508,
                "wheel_base": 0.3407,
                # PATCH (isaac-social-nav): slight lift so sphere tires settle onto ground
                # instead of spawning in penetration (z=0 put tire bottoms exactly at floor).
                "spawn_z_lift": 0.002,
            },
        }

        if robot_name not in robot_configs:
            raise ValueError(f"Unsupported robot_name: {robot_name}")

        robot_config = robot_configs[robot_name]
        robot_init = _load_robot_init_pose(hunav_config)
        spawn_z = robot_init["z"] + float(robot_config.get("spawn_z_lift", 0.0))
        spawn_position = [
            robot_init["x"],
            robot_init["y"],
            spawn_z,
        ]
        spawn_orientation = _yaw_to_orientation(robot_init["h"])
        
        # Handle absolute vs relative paths for robot USD files
        if os.path.isabs(robot_config["usd_relative_path"]):
            # Absolute path (for custom robots like carter_ROS)
            robot_path = robot_config["usd_relative_path"]
        else:
            # Relative path (for built-in Isaac Sim robots)
            robot_path = os.path.join(assets_root_path, robot_config["usd_relative_path"])

        # Add robot to world
        robot_prim_path = f"/World/{robot_config['name']}"
        # ORIGINALLY (upstream v2.0): always WheeledRobot + DifferentialController at [0,0,0].
        # PATCH (isaac-social-nav): Stretch uses chassis_only kinematic base; optional robot_init_pose.
        # stretch_wheeled keeps PhysX WheeledRobot + DifferentialController (walls collide).
        if robot_config.get("drive") == "chassis_only":
            self._chassis_drive = True
            self.robot = ChassisDriveRobot(
                prim_path=robot_prim_path,
                usd_path=robot_path,
                position=spawn_position,
                orientation=spawn_orientation,
            )
            self.diff_controller = None
        else:
            self._chassis_drive = False
            self.robot = self.world.scene.add(
                WheeledRobot(
                    prim_path=robot_prim_path,
                    name="Robot",
                    wheel_dof_names=robot_config["wheel_dof_names"],
                    create_robot=True,
                    usd_path=robot_path,
                    position=spawn_position,
                    orientation=spawn_orientation,
                )
            )

            # Create differential drive controller
            self.diff_controller = DifferentialController(
                name="diff_drive_controller",
                wheel_radius=robot_config["wheel_radius"],
                wheel_base=robot_config["wheel_base"],
            )

        # ROS2 cmd_vel subscriber
        self.cmd_lin = 0.00
        self.cmd_ang = 0.00
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )

        # Setup HuNavManager
        self.hunav = HuNavManager(
            node=self,
            world=self.world,
            config_file_path=hunav_config,
            robot_prim_path=robot_prim_path,
            robot=self.robot,
        )

        self.create_ros_clock_action_graph()

        self.hunav.initialize_agents()
        self.hunav.initialize_hunav_nodes()

    def _signal_handler(self, signum, frame):
        print("\n\nCaught shutdown signal, closing app and stopping hunav nodes...\n\n")
        self.hunav.close_hunav_nodes()
        simulation_app.close()

    def _cmd_vel_callback(self, msg):
        self.cmd_lin = msg.linear.x
        self.cmd_ang = msg.angular.z

    def _on_physics_step(self, dt: float):
        """
        Called automatically by PhysX each physics frame.
        """
        self.hunav.send_agents_msg()

    def create_ros_clock_action_graph(self, graph_path="/World/ROS2"):
        try:
            keys = og.Controller.Keys
            graph_params = {
                # Create the necessary nodes.
                keys.CREATE_NODES: [
                    # Node for generating a tick on playback.
                    ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                    # Node for reading the simulation time.
                    (
                        "isaac_read_simulation_time",
                        "isaacsim.core.nodes.IsaacReadSimulationTime",
                    ),
                    # Node to create a ROS2 context.
                    ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                    # Node to publish the clock over ROS2.
                    ("ros2_publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                # Connect outputs to inputs:
                keys.CONNECT: [
                    # Connect context output to the publish clock's context input.
                    (
                        "ros2_context.outputs:context",
                        "ros2_publish_clock.inputs:context",
                    ),
                    # Connect tick output to publish clock's execIn.
                    (
                        "on_playback_tick.outputs:tick",
                        "ros2_publish_clock.inputs:execIn",
                    ),
                    # Connect simulation time output to publish clock's timeStamp.
                    (
                        "isaac_read_simulation_time.outputs:simulationTime",
                        "ros2_publish_clock.inputs:timeStamp",
                    ),
                ],
                keys.SET_VALUES: [
                    # For the simulation time node.
                    ("isaac_read_simulation_time.inputs:resetOnStop", True),
                    ("isaac_read_simulation_time.inputs:swhFrameNumber", 0),
                    # For the ROS2PublishClock node.
                    ("ros2_publish_clock.inputs:nodeNamespace", ""),
                    ("ros2_publish_clock.inputs:qosProfile", ""),
                    ("ros2_publish_clock.inputs:queueSize", 10),
                    ("ros2_publish_clock.inputs:timeStamp", 0.0),
                    ("ros2_publish_clock.inputs:topicName", "clock"),
                    # For the ROS2Context node.
                    ("ros2_context.inputs:useDomainIDEnvVar", True),
                    ("ros2_context.inputs:domain_id", 0),
                ],
            }
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                graph_params,
            )
            print(f"Successfully created ROS_Clock action graph at {graph_path}")
        except Exception as e:
            print(f"Error creating ROS_Clock action graph: {e}")

    def run(self):
        self.world.reset()
        # ---------------------------------------------------------------------------
        # ORIGINALLY (upstream v2.0):
        # self.physx_interface = omni.physx.acquire_physx_interface()
        # Had to be patched because Isaac 6.0 renamed the API to get_physx_interface
        # (AttributeError: module 'omni.physx' has no attribute 'acquire_physx_interface').
        # ---------------------------------------------------------------------------
        # PATCH (isaac-social-nav): prefer get_physx_interface, fall back to acquire_*
        # so physics step subscription works on Isaac 6.0 (and older if present).
        # ---------------------------------------------------------------------------
        _physx = getattr(omni.physx, "get_physx_interface", None) or getattr(
            omni.physx, "acquire_physx_interface", None
        )
        self.physx_interface = _physx()
        self.physx_sub = self.physx_interface.subscribe_physics_step_events(
            self._on_physics_step
        )
        self.hunav.send_agents_msg()
        while simulation_app.is_running():
            self.world.step(render=True)
            if self._chassis_drive:
                dt = self.world.get_physics_dt()
                self.robot.apply_cmd_vel(self.cmd_lin, self.cmd_ang, dt)
            else:
                wheel_action = self.diff_controller.forward([self.cmd_lin, self.cmd_ang])
                self.robot.apply_wheel_actions(wheel_action)
