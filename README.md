# Forked for isaac-social-nav (Isaac Sim 6.0.1 + ROS 2 Jazzy)

Fork of [robotics-upo/Hunav_isaac_wrapper](https://github.com/robotics-upo/Hunav_isaac_wrapper) (`v2.0`) for the [isaac-social-nav](https://github.com/osamuzahid/isaac-social-nav) dissertation stack.

**Why forked:** upstream targets Isaac Sim 4.5 / ROS 2 Humble / Ubuntu 22.04. This branch ports the wrapper onto **Isaac Sim 6.0.1** and **ROS 2 Jazzy** (Ubuntu 24.04).

**What changed vs upstream (branch `isaac-6.0-jazzy`):**
- Remapped CDN asset paths (Biped_Setup, Jetbot/Create3/Carter USDs) that 404 on Isaac 6.0 content
- AnimGraph `speed` variable authored as uniform float (fixes compile / type-mismatch on 6.0)
- PhysX subscribe via `get_physx_interface` (old `acquire_physx_interface` removed)
- Laptop/debug `SimulationApp` profiles (`HUNAV_ISAAC_PROFILE=debug|laptop|default|lab`) via `sim_app_config.py`
- Fixed `ros2 run … hunav_isaac_launcher` resource paths for this colcon layout; added `empty_world_agents` preset
- CUCR museum v1: `museum.usd` (Isaac `asset_converter` + Z-up compose; `MUSEUM_Z_OFFSET=0.1` so walls meet floor / base lidar — Validated **#47**), `museum_agents.yaml`, `maps/museum.png`; rebuild via `tools/isaac_convert_museum.py`
- Museum mixed behaviors: `museum_behaviors` preset (Regular / Curious / Scared / Surprised / Threatening)
- CUCR office: `office.usd` is ServiceSim (`cucr_worlds_office`), not the stock HuNav bake — `tools/README_office_port.md`
- CUCR bookstore: `bookstore.usd` (AWS RoboMaker retail; convert `tools/isaac_convert_bookstore.py`) + `bookstore_behaviors` disjoint aisle loops — `tools/README_bookstore_port.md`
- CUCR house_museum / small_house / small_warehouse: remaining `cucr_worlds` ports (GUI-good; house_museum roofs off + sunset sky kept) — `tools/README_remaining_ports.md`
- Lab Reachy: `--robot reachy` (Zuuu + full-kit torso; kinematic `chassis_only` + `/scan` + dual RGB — Validated **#49/#59/#60/#77**). Hospital occupancy hop: `tools/drive_reachy_waypoints.py` `(5,0)`→`(5,-8)`.
- Lab Stretch: `--robot stretch` (kinematic chassis) and `--robot stretch_wheeled` (PhysX diff-drive; flat-ground smoke PASS)
- Stretch Nav2: `KinematicNavPublisher` (`world→map→odom→base_link` + `/odom`); museum A10 plaza **#69**; plaza `/cmd_vel` A* **#68**; `museum_eval` static crowd **#67**. Reachy uses the same publisher (`lidar_link` + `laser` alias) — **#77**.
- Lab Franka: `--robot franka` (CDN `FrankaPanda`, parked `drive: static`) + stock TF/`joint_states`
- Lab sensors: `lab_robot_sensors.py` — Stretch `/scan` (RTX 2D) + `/imu` + parked joints; RGB-D via `HUNAV_LAB_CAMERAS=1` under `camera_color_optical_frame` (Validated **#46**); museum `/scan` wall rings after Z fix (**#47**); smoke `tools/lab_robot_sensor_smoke.py`
- Agent walk on Isaac 6: People biped retarget source, file-referenced retarget clips, SFM goals wiring, museum free-space goals + collider/ray fixes — discovery log **[docs/ANIMATION_LOG.md](docs/ANIMATION_LOG.md)**; current architecture (parent repo) **[docs/PEOPLE_MOTION.md](https://github.com/osamuzahid/isaac-social-nav/blob/master/docs/PEOPLE_MOTION.md)**
- In-source notes: search `ORIGINALLY` / `PATCH (isaac-social-nav)`

Parent project docs: [isaac-social-nav](https://github.com/osamuzahid/isaac-social-nav) (`README.md`, `docs/ENVIRONMENT.md` Handoff, `docs/TROUBLESHOOTING.md`).

---

# **HuNav Isaac Wrapper**

A standalone simulation wrapper for **NVIDIA Isaac Sim**, integrating the **Human Navigation Simulator ([HuNavSim](https://github.com/robotics-upo/hunav_sim))** with **physics-based animations** and **ROS 2 integration**.

---

### **Work in Progress**

This repository is actively developed and subject to improvements.

### **Tested Configurations**

- **This fork (`isaac-6.0-jazzy`):** ROS 2 Jazzy, Isaac Sim 6.0.1, Ubuntu 24.04
- **Upstream `v2.0`:** ROS 2 Humble, Isaac Sim 4.5, Ubuntu 22.04 LTS

## **Overview**

**HuNav Isaac Wrapper** is a modular simulation framework that integrates the [HuNavSim](https://github.com/robotics-upo/hunav_sim) human navigation simulator into **NVIDIA Isaac Sim**, enabling realistic multi-agent behavior with physics-based animation and **ROS 2** interoperability.

It supports both **ROS 2 teleoperation** and **autonomous navigation (Nav2)**, world loading, dynamic agent configuration, and multiple robot models.

*This wrapper is built for research in **human-robot interaction**, **social navigation**, and **simulation-based validation of social navigation policies**.*

---

## **Features**

- **ROS2 Workspace Structure:**
 - Complete ROS2 workspace that can be cloned and built directly with `colcon build`

- **Modular Architecture:**
 - `main.py`: Interactive launcher providing a command-line interface for configuration selection (agent files, worlds, robots) and simulation startup
 - `world_builder.py`: Loads USD world files
 - `hunav_manager.py`: Handles agent creation, communication with HuNavSim services, and manages physics, animations, and obstacle detection
 - `teleop_hunav_sim.py`: Manages HuNavSim initialization, updates agent states, and handles the ROS 2 /cmd_vel interface for robot control
 - `animation_utils.py`: Utilities for AnimationGraph setup and retargeting

- **Enhanced Agent Configuration:**
 - YAML files (in `scenarios/`) define agent spawn positions, navigation goals, SFM parameters, and behavior profiles
 - Backward compatible with existing configuration files

- **Animation System:**
 - **AnimationGraph-based** blending for smooth walk/idle transitions
 - Driven by agent velocity, allowing dynamic switching between walk and idle states
 - Supports animation **retargeting**, applying a single set of animations to different characters via **USD SkelAnimation** and the **Omni Anim Retargeting extension**

- **Multiple Robot Models:**
 - Includes `jetbot`, `create3`, `carter`, `carter_ROS`, lab Stretch (`stretch`, `stretch_wheeled`), lab Franka (`franka`), and lab Reachy (`reachy`, Zuuu kinematic chassis)

- **ROS 2 Navigation (Nav2) Support:**
 - Dissertation path: kinematic **Stretch** (`tools/nav2_smoke/run_stretch_nav2_smoke.sh`, museum A10 plaza)
 - Optional tooling: **Carter** (`carter_ROS`) using the **ROS 2 Nav2** stack

- **Social Simulation & Teleoperation:**
 - Real-time robot control via `/cmd_vel`
 - Socially-aware agent movement via **HuNavSim** integration

- **Obstacle Detection:**
 - Uses **PhysX raycasts** (in `hunav_manager.py`) for detecting obstacles and informing **HuNavSim** navigation logic

---

## **Requirements**

- **Ubuntu 22.04 LTS**
- [**HuNavSim**](https://github.com/robotics-upo/hunav_sim)
- [**NVIDIA Isaac Sim (Workstation Installation)**](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)

- **Python 3.8+**

- **ROS 2 **[Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)****

---

## **Setup Guide**

### 1. Repository Setup

#### **Option 1: Direct Clone (Recommended)**

This project is structured as a complete ROS2 workspace:

```bash
# Clone the repository
git clone https://github.com/robotics-upo/Hunav_isaac_wrapper
cd Hunav_isaac_wrapper

# Install ROS2 dependencies
sudo apt install ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-tf2-ros

# Install Python dependencies
pip install pyyaml numpy matplotlib

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

#### **Option 2: Add to Existing ROS2 Workspace**

If you want to integrate this into an existing ROS2 workspace:

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/robotics-upo/Hunav_isaac_wrapper.git

# Move the package contents to your workspace
cp -r hunav_isaac_wrapper/src/* .
rm -rf hunav_isaac_wrapper

# Build your workspace
cd ~/your_ros2_ws
colcon build --packages-select hunav_isaac_wrapper
source install/setup.bash
```

#### **Additional Dependencies**

You'll also need to install HuNavSim in case you haven't already:

```bash
# In your ROS2 workspace src directory
git clone https://github.com/robotics-upo/hunav_sim.git
cd .. && colcon build
```

### 2. Isaac Sim Installation

Make sure you have **NVIDIA Isaac Sim** installed. Follow the [Isaac Sim Installation Guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html).

### 3. Configure Isaac Sim Extensions

To ensure all required dependencies are active, replace the existing `isaacsim.exp.base.kit` file from your Isaac Sim installation with the one provided in this repository:

```bash
# After cloning this repository
cp src/isaacsim.exp.base.kit ~/isaacsim/apps/
```

**Important**: This file ensures that essential extensions (e.g., animation retargeting, ROS 2 bridge) are preloaded at startup.

### 4. ROS 2 Setup

Ensure ROS 2 Humble is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

### 5. Configure Your Scene, Agents, and Robot

The simulation setup is configured through the interactive launcher, which provides a menu-driven interface to select the world, agent configuration, and robot model.

#### Scene Setup

The repository includes multiple pre-configured USD files in `src/worlds/`:

- `warehouse.usd`: Industrial layout with shelves and various obstacles
- `hospital.usd`: Medical environment with corridors and rooms
- `office.usd`: CUCR ServiceSim office (not stock HuNav bake)
- `empty_world.usd`: A minimal open environment for testing
- `museum.usd`: CUCR `cucr_worlds_museum` layout (Isaac asset_converter; pair with `museum_agents.yaml`)
- `bookstore.usd`: CUCR `cucr_worlds_bookstore` (AWS RoboMaker retail; pair with `bookstore_behaviors.yaml`)
- `house_museum.usd`: CUCR `cucr_worlds_house_museum` (Natural Museum Cardiff; pair with `house_museum_behaviors.yaml`)
- `small_house.usd`: CUCR `cucr_worlds_small_house` (AWS residential; ceiling skipped)
- `small_warehouse.usd`: CUCR `cucr_worlds_small_warehouse` (AWS warehouse; roof skipped; **not** stock `warehouse.usd`)

#### Agents Configuration

Each world is paired with a YAML file in `src/scenarios/` that defines the HuNavSim agents:

- **Available configuration files:**
 - `warehouse_agents.yaml` for `warehouse.usd`
 - `hospital_agents.yaml` for `hospital.usd`
 - `office_agents.yaml` / `office_behaviors.yaml` for `office.usd` (CUCR ServiceSim)
 - `empty_world_agents.yaml` for `empty_world.usd`
 - `museum_agents.yaml` for `museum.usd`
 - `bookstore_agents.yaml` / `bookstore_behaviors.yaml` for `bookstore.usd`
 - `house_museum_agents.yaml` / `house_museum_behaviors.yaml` for `house_museum.usd`
 - `small_house_agents.yaml` / `small_house_behaviors.yaml` for `small_house.usd`
 - `small_warehouse_agents.yaml` / `small_warehouse_behaviors.yaml` for `small_warehouse.usd`

- **Each YAML file lets you configure:**
 - **Initial pose**: Define starting positions of each agent.
 - **Goals**: Set destination coordinates or waypoints per agent.
 - **SFM weights**: Tune the social force model for realistic crowd behavior.
 - **Behavior type**: Choose how agents behave.

**Always pair the world with its corresponding agent YAML to avoid misaligned goals or initial agent positions**.

#### Robot Configuration

The interactive launcher will prompt you to select your desired robot:
- `jetbot`, `create3`, `carter`, or `carter_ROS`

**Note:** For `carter_ROS`, make sure to unzip the `nova_carter_ros2_sensors` package located in `src/config/robots/`.

**Carter** remains optional Nav2 tooling. Dissertation Stretch Nav2: `~/Desktop/run-museum-stretch-nav2.sh` (A10 plaza).

### 6. Launch the Simulation

You can launch the simulation using two methods:

#### Method 1: Shell Script

```bash
cd ~/Hunav_isaac_wrapper # Navigate to the repository root
# Make sure the script is executable
chmod +x launch_hunav_isaac.sh
# Launch the simulation
./launch_hunav_isaac.sh
```

#### Method 2: ROS2 Run Command

```bash
# Interactive mode
ros2 run hunav_isaac_wrapper hunav_isaac_launcher

# With custom arguments
ros2 run hunav_isaac_wrapper hunav_isaac_launcher --config warehouse_agents.yaml --robot carter_ROS --batch
```

Both methods provide the same functionality and will:

- Start the interactive launcher with menu-driven configuration (when no scenario specified)
- Guide you through selecting agent configuration and robot model
- Load the specified world and spawn agents based on your selections
- Initialize physics, animations, and ROS 2 integration

### 7. Teleoperation and Navigation

#### Teleoperation

Use ROS 2 to publish Twist messages to `/cmd_vel` for direct robot control:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.1}}'
```

Kinematic Stretch/Reachy (`Physics=none`) **hold the last twist** until a new one arrives, including zeros. `ros2 topic pub … -r 10` keeps moving after Ctrl-C until you publish a zero twist. Raw `/cmd_vel` ghosts walls; occupancy / Nav2 / ESC keep the robot in halls.

#### ROS 2 Navigation (Nav2)

Dissertation robot is kinematic **Stretch** (museum A10 plaza). Desktop: `~/Desktop/run-museum-stretch-nav2.sh`. Params: `tools/nav2_smoke/nav2_stretch_params.yaml`. Viewport follow is `--frame-robot` only.

Optional **Carter** (`carter_ROS`) tooling:

1. Ensure the simulation is running
2. In a separate terminal, launch the navigation stack:

 ```bash
 ros2 launch carter_navigation carter_navigation.launch.py \
 params_file:="src/config/navigation_params/carter_navigation_params.yaml" \
 map:="src/maps/warehouse.yaml"
 ```

## **Troubleshooting**

### Automated Setup Issues

If you encounter issues with the manual setup steps, you can use the automated setup script:

 ```bash
 # Make the script executable
 chmod +x setup_workspace.sh

 # Run the setup script
 ./setup_workspace.sh
 ```

This script will:

- Verify ROS2 is properly sourced
- Check for required dependencies (`hunav_msgs`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`)
- Automatically build the package if in a colcon workspace
- Provide guidance for workspace creation if needed
- Display usage examples for different launch methods

### ROS 2 Connectivity

Make sure that your ROS 2 Humble installation is sourced:

 ```bash
 source /opt/ros/humble/setup.bash
 ```

If `carter_navigation` package is not recognized, follow these steps:

1. Clone the [IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces.git) repository:

 ```bash
 git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
 ```

2. Build the ROS 2 humble workspace:

 ```bash
 cd IsaacSim-ros_workspaces/humble_ws
 colcon build
 ```

3. Source the workspace in your `.bashrc`:

 ```bash
 source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
 ```

### Agent Configuration Issues

- If agents appear horizontal rather than vertical, adjust the rotation applied in the script (e.g., modify the quaternion calculation in `hunav_manager.initialize_agents()`'s `init_rot` parameter and/or `hunav_manager._update_agents()`).

### Retargeting Errors (*)

- Verify that the default source biped prim is correctly loaded at `/World/biped_demo`.
- Ensure that the target agents have a valid **skeleton** (use the `findSkeletonPath` method for debugging).
- Confirm that the extension `omni.anim.retarget.core` is enabled.

### AnimationGraph Issues

- If AnimationGraph is not applying correctly, verify that it is correctly assigned to the agent's **SkelRoot** and that transformations are applied properly.

---

## Acknowledgments

This work is carried out as part of the **HunavSim 2.0** project, _“A Human Navigation Simulator for Benchmarking Human-Aware Robot Navigation”_, supported under the [**euROBIN 2nd Open Call – Technology Exchange Programme**](https://www.eurobin-project.eu/index.php/showroom/news/47-2nd-call-eurobin-technology-exchange-programme) (**euROBIN_2OC_2**), funded by the **European Union's Horizon Europe** research and innovation programme under grant agreement **No. 101070596**.

<p align="left">
 <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/84/European_Commission.svg/300px-European_Commission.svg.png" width="160"/>
 &nbsp;&nbsp;&nbsp;
 <img src="https://www.eurobin-project.eu/images/2025/03/15/eurobin_logo-_payoff.png" alt="euROBIN Logo" width="160"/>
</p>

