# 🌿 Greenhouse Robotic Arm Harvesting System

A vision-guided mobile manipulator simulated in ROS 2 + Gazebo, designed for
autonomous greenhouse fruit harvesting. The system drives along a row, detects
fruit with an RGB-D camera, plans collision-free motion with MoveIt 2, picks
each fruit, and sorts it by size into the correct bin.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Modules](#modules)
- [Hardware Setup](#hardware-setup-simulation)
- [Dependencies](#dependencies)
- [Quick Start](#quick-start)
- [Build](#build)
- [Running the Simulation](#running-the-simulation)
- [Manual Commands](#manual-commands)
- [Project Structure](#project-structure)
- [Simulation & Evaluation](#simulation--evaluation)
- [Known Limitations & Future Work](#known-limitations--future-work)
- [Team](#team)

---

## Overview

Greenhouse farming relies heavily on manual labour for repetitive tasks such as
harvesting and sorting. This project addresses those inefficiencies by designing
and simulating a mobile manipulator capable of:

- Detecting and localising fruits in 3D space using an RGB-D camera
- Driving precisely between rows using closed-loop base control
- Planning collision-free trajectories with MoveIt 2
- Grasping fruits with a parallel-jaw end-effector
- Sorting harvested produce by size into separate containers

The system is validated entirely in simulation, with all robotics components
(arm, base, gripper, camera, lidar) modelled in a Gazebo greenhouse world.

---

## System Architecture

The system is built on **ROS 2 Jazzy** and simulated in **Gazebo Harmonic**.
Application logic lives in three custom nodes that coordinate via ROS 2 topics,
while the heavy lifting (motion planning, controller execution, simulation) is
delegated to standard ROS 2 components.

```
                    ┌────────────────────────┐
                    │   robot_coordinator    │  FSM: drive → scan → pick → repeat
                    └───┬──────────────┬─────┘
            /cmd_vel    │              │   /arm/task
                        ▼              ▼
                ┌──────────────┐  ┌────────────────┐
                │   Gazebo     │  │ arm_controller │
                │ (base, arm,  │  │  (MoveIt API)  │
                │  sensors)    │  └────────┬───────┘
                └──────┬───────┘           │ MoveGroup
                       │ /camera/...       ▼ action
                       ▼            ┌──────────────┐
                ┌──────────────┐    │  move_group  │
                │fruit_detector│    │  (MoveIt 2)  │
                └──────┬───────┘    └──────────────┘
                       │ /fruit/pose
                       │ /fruit/diameter
                       ▼
                  robot_coordinator (back to FSM)
```

---

## Modules

### 1. Vision-Based Perception (`fruit_detector`)

Subscribes to the RGB and depth image streams from the simulated RealSense
camera, segments red fruits via HSV thresholding, and computes 3D position in
the base-link frame using the depth channel and the camera-to-base transform:

```
P_base = T_camera→base · P_camera
```

For each detection, the node publishes:
- `/fruit/pose` (`geometry_msgs/PoseStamped`) — fruit position in base_link
- `/fruit/diameter` (`std_msgs/Float32`) — estimated fruit diameter in metres

### 2. High-Level Coordination (`robot_coordinator`)

A finite-state machine that drives the harvest cycle:

```
INIT → STARTUP_HOME → CAMERA_SETTLE → DRIVING → SCANNING → POST_SCAN
                                          ▲                    │
                                          └────  PICKING  ◀────┘
                                                    │
                                                    ▼
                                                  DONE
```

- **DRIVING** uses a P-controller on the base. Velocity is proportional to
  remaining distance, with min/max clamping. The controller reads the robot's
  true world pose directly from Gazebo (bypassing wheel odometry, which suffers
  from slip-induced drift).
- **SCANNING** waits a fixed window for the detector to publish a reachable
  fruit pose.
- **PICKING** delegates to the arm controller and waits for `picked` / `failed`
  status before resuming the drive.

### 3. Arm Control & Harvest Pipeline (`arm_controller`)

A MoveIt 2 client that converts task strings (`pick_with_size:x,y,z,d`, `home`,
`place:large`, etc.) into MoveGroup goals. Each successful pick runs a 5-step
sequence:

1. **PRE-GRASP** — plan to a 10 cm stand-off pose in front of the fruit (OMPL).
2. **APPROACH** — plan from pre-grasp to the fruit pose (OMPL).
3. **Verify** — read the gripper TF and check Euclidean distance to the target.
   If error exceeds tolerance (default 10 cm), the pick aborts before deleting
   the fruit.
4. **LIFT** — plan back to the pre-grasp pose. A red sphere is spawned at the
   gripper and tracked at 20 Hz via Gazebo's `set_pose` service, giving the
   visual impression of carrying the fruit.
5. **HOME → BIN** — plan to the joint-space HOME pose, then to the size-
   appropriate bin (`BIN_LARGE` or `BIN_SMALL`). The carried fruit is removed
   and replaced with a dynamic fruit that falls into the bin.

Per-axis grasp compensations (`X_COMPENSATION`, `Y_COMPENSATION`,
`Z_COMPENSATION`) are exposed at the top of the file to tune for systematic
offsets between perception and the arm's TF chain.

### 4. Motion Planning (MoveIt 2)

All Cartesian and joint-space planning is performed by MoveIt 2 using its
default OMPL pipeline. Trajectory execution goes through MoveIt's controller
manager and the joint trajectory controller defined in `controllers.yaml`.
Two parameters must be set at every run to allow long-duration trajectories:

```
/move_group  trajectory_execution.allowed_execution_duration_scaling = 100.0
/move_group  trajectory_execution.allowed_goal_duration_margin      = 60.0
```

### 5. Sorting by Size

Fruits are classified by the perceived diameter against `SIZE_THRESHOLD_M`
(default 6 cm). Larger fruits are routed to the BIG bin; smaller ones to the
SMALL bin. Bin positions are defined as world-frame offsets relative to the
robot's current pose.

---

## Hardware Setup (Simulation)

| Component | Description |
|-----------|-------------|
| Robotic Arm | 5-DOF revolute arm (joints `Revolute_14` to `Revolute_18`) |
| Mobile Base | 4-wheel platform with differential-style velocity command (`/cmd_vel`) |
| End-Effector | Parallel-jaw gripper (prismatic joint `Slider_20`, 0–8 cm) |
| Camera | RGB-D camera with adjustable pan joint |
| Lidar | 360° planar lidar |
| Robot Description | URDF (assembled from xacros) with collision geometry |
| Environment | Gazebo greenhouse world with fruit-bearing crop rows and two sorting bins |

![Gazebo Simulation](simulation_preview.jpg)

---

## Dependencies

### Operating system

| | |
|---|---|
| OS | Ubuntu 24.04 (Noble Numbat) |
| Shell | bash |

### Core software

| Package | Version |
|---------|---------|
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| MoveIt 2 | Jazzy |
| Python | 3.12 |

### ROS 2 packages

| Package |
|---------|
| `ros-jazzy-moveit` |
| `ros-jazzy-ros-gz-bridge` |
| `ros-jazzy-ros-gz-sim` |
| `ros-jazzy-ros-gz-interfaces` |
| `ros-jazzy-ros2-control` |
| `ros-jazzy-ros2-controllers` |
| `ros-jazzy-gz-ros2-control` |
| `ros-jazzy-xacro` |
| `ros-jazzy-robot-state-publisher` |
| `ros-jazzy-joint-state-publisher` |
| `ros-jazzy-rqt-image-view` |
| `ros-jazzy-rqt-graph` |
| `ros-jazzy-tf2-tools` |

### System Python packages

| Package |
|---------|
| `python3-numpy` |
| `python3-opencv` |
| `python3-yaml` |

---

## Quick Start

Once all listed dependencies are installed, follow these steps in order from a
fresh terminal. Each command assumes the previous one succeeded — don't skip
ahead.

### 1. Clone into a fresh ROS 2 workspace

```bash
mkdir -p ~/ros2_gw/src
cd ~/ros2_gw/src
git clone https://github.com/YOUR_USERNAME/chaleBOT.git chale_bot
```

> Replace `YOUR_USERNAME` with the actual GitHub user/org.

### 2. Generate the world

The Gazebo world is built from a xacro template — it must be expanded once
before the first build:

```bash
cd ~/ros2_gw/src/chale_bot/chaleBOT_description/worlds
xacro greenhouse.world.xacro > greenhouse.world
```

### 3. Build with colcon

```bash
cd ~/ros2_gw
colcon build --packages-select chaleBOT_description chaleBOT_moveIt_config greenhouse_robot
```

### 4. Source the workspace in every new terminal

```bash
source ~/ros2_gw/install/setup.bash
```

Optional — auto-source on every new shell:
```bash
echo "source ~/ros2_gw/install/setup.bash" >> ~/.bashrc
```

### 5.  Strong reset before launching

If you have ROS 2 or Gazebo processes from a previous run still alive, kill them:
```bash
pkill -9 -f "gz"; pkill -9 -f "ruby"
pkill -9 -f "rviz2"; pkill -9 -f "move_group"
pkill -9 -f "arm_controller"; pkill -9 -f "robot_coordinator"
pkill -9 -f "fruit_detector"
pkill -9 -f "parameter_bridge"
pkill -9 -f "robot_state_publisher"
pkill -9 -f "spawner"
sleep 5
```

### 6. Launch the demo 

Open six fresh terminals. Source the workspace in each one
(`source ~/ros2_gw/install/setup.bash`), then run them **in the order shown**
and wait for each "ready" message before moving on to the next.

**Terminal 1 — Gazebo + ros2_control**
```bash
ros2 launch chaleBOT_description gazebo.launch.py
```
Wait for Gazebo to fully open and the robot to spawn. In another terminal,
confirm that all controllers are active:
```bash
ros2 control list_controllers
```
All four controllers must show `active` before continuing.

**Terminal 2 — MoveIt 2 + RViz**
```bash
ros2 launch chaleBOT_moveIt_config move_group.launch.py
```
Wait for `You can start planning now!` in the log.

**Side terminal — set MoveIt trajectory parameters (required every run)**
```bash
ros2 param set /move_group trajectory_execution.allowed_execution_duration_scaling 100.0
ros2 param set /move_group trajectory_execution.allowed_goal_duration_margin 60.0
```

**Terminal 3 — arm controller**
```bash
ros2 run greenhouse_robot arm_controller
```
Wait for `MoveIt connected. Registry has N fruit(s). ... Ready for tasks.`

**Terminal 4 — fruit detector**
```bash
ros2 run greenhouse_robot fruit_detector
```

**Terminal 5 — camera viewer (optional)**
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/camera/color/image_raw` from the dropdown.

**Terminal 6 — robot coordinator (starts the demo)**
```bash
ros2 run greenhouse_robot robot_coordinator
```

The harvest sequence runs automatically: drive → scan → pick → sort → repeat.

### Common pitfalls

- **Workspace not sourced** — every new terminal needs
  `source ~/ros2_gw/install/setup.bash`. If `ros2 run greenhouse_robot ...`
  reports "package not found", the source was skipped.
- **Pip-installed NumPy / OpenCV** — pip installs fight with the apt-shipped
  versions and cause segfaults in `fruit_detector`. Use only the apt packages
  in the Dependencies table (`python3-numpy`, `python3-opencv`, `python3-yaml`).
- **Skipped trajectory parameters** — without the two `param set` calls in
  Step 6 (side terminal), arm motions abort with timeouts.
- **Wrong launch order** — the arm controller (T3) needs MoveIt (T2) to be
  fully up first. Coordinator (T6) needs both T3 and the detector (T4) up.
  Always follow the numbered order.
- **Stale build artifacts after pulling new code** — if `git pull` brings new
  files, rebuild before relaunching:
  ```bash
  cd ~/ros2_gw && colcon build --packages-select chaleBOT_description chaleBOT_moveIt_config greenhouse_robot && source install/setup.bash
  ```

---

## Build

```bash
# 1. Generate the world from the xacro template
cd ~/ros2_gw/src/chale_bot/chaleBOT_description/worlds
xacro greenhouse.world.xacro > greenhouse.world

# 2. Build the packages with colcon
cd ~/ros2_gw
colcon build --packages-select chaleBOT_description chaleBOT_moveIt_config greenhouse_robot
source install/setup.bash
```

---

## Running the Simulation

The full demo requires six terminals. Each must have the workspace sourced
(`source ~/ros2_gw/install/setup.bash`).

**Terminal 1 — Gazebo + ros2_control**
```bash
ros2 launch chaleBOT_description gazebo.launch.py
```

**Terminal 2 — MoveIt 2 + RViz**
```bash
ros2 launch chaleBOT_moveIt_config move_group.launch.py
```

**Side terminal — set MoveIt trajectory parameters (required every run)**
```bash
ros2 param set /move_group trajectory_execution.allowed_execution_duration_scaling 100.0
ros2 param set /move_group trajectory_execution.allowed_goal_duration_margin 60.0
```

**Terminal 3 — arm controller**
```bash
ros2 run greenhouse_robot arm_controller
```

**Terminal 4 — fruit detector**
```bash
ros2 run greenhouse_robot fruit_detector
```

**Terminal 5 — camera viewer**
```bash
ros2 run rqt_image_view rqt_image_view
```

**Terminal 6 — robot coordinator (starts the demo)**
```bash
ros2 run greenhouse_robot robot_coordinator
```

---

## Manual Commands

While the system is running, the arm can be driven directly via the `/arm/task`
topic.

| Command | Effect |
|---------|--------|
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'home'}"` | Move arm to HOME pose |
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'goto:0.3,-0.5,0.7'}"` | Move end-effector to Cartesian pose in base_link |
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'pick_with_size:0.3,-0.5,0.7,0.07'}"` | Run a full pick at the given pose with the given diameter (m) |
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'place:large'}"` | Move arm over the BIG bin |
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'place:small'}"` | Move arm over the SMALL bin |
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'open'}"` | Open gripper |
| `ros2 topic pub --once /arm/task std_msgs/msg/String "{data: 'close'}"` | Close gripper |
| `ros2 topic echo /arm/status` | Watch arm status messages |
| `ros2 run rqt_graph rqt_graph` | View the live node graph |
| `ros2 run tf2_ros tf2_echo base_link gripper_clamp_1` | Echo end-effector pose |

---

## Project Structure

```
src/chale_bot/
├── chaleBOT_description/      # URDF, world xacro, bridge config, launch files
│   ├── urdf/                  # Robot URDF and xacros
│   ├── worlds/                # Greenhouse world template
│   ├── parameters/            # ros_gz_bridge configuration
│   └── launch/                # Gazebo launch
├── chaleBOT_moveIt_config/    # MoveIt 2 configuration
│   ├── config/                # kinematics.yaml, controllers, joint limits
│   └── launch/                # move_group launch
└── greenhouse_robot/          # Custom application nodes
    ├── greenhouse_robot/
    │   ├── arm_controller.py      # MoveIt client, harvest pipeline
    │   ├── fruit_detector.py      # HSV + depth-based detection
    │   └── robot_coordinator.py   # FSM that drives the demo
    └── config/
        └── fruit_registry.yaml    # Ground-truth fruit positions
```

---

## Simulation & Evaluation

The pipeline is tested in Gazebo. Performance is tracked using:

- **Harvest success rate** — fraction of attempted picks that complete
- **Average cycle time per fruit** — drive + scan + pick + sort
- **Grasp position error** — Euclidean distance between gripper TF and the
  perception-reported fruit position at the grasp moment (logged each pick)
- **Detection accuracy** — fraction of fruits in the row that the detector
  reports during a scan window
- **Sorting accuracy** — fraction of fruits routed to the correct bin

---

## Known Limitations & Future Work

This is a simulation-focused project. The following items are deferred as
future work:

- **Suction-based end-effector** — the current parallel-jaw gripper is modelled
  kinematically. A vacuum gripper with `DetachableJoint` and contact-based
  attach/detach is the natural next step for soft-fruit harvesting realism.
- **Yaw-aware base motion** — the harvest pipeline assumes the rover's heading
  is fixed (yaw = 0) when transforming between base_link and world frames.
  Supporting rotational motion requires rotating perception outputs by the
  current world-yaw.
- **Wheel-slip-resistant odometry** — Gazebo's true robot pose is currently
  read via the `gz` CLI to bypass odom drift. A proper visual-inertial or
  lidar odometry would generalise to hardware.
- **Force-sensitive grasping** — the current grasp does not measure contact
  forces. Adding a 6-axis F/T sensor at the wrist would enable gentle-grasp
  behaviours for delicate produce.
- **Ripeness classification** — colour segmentation currently classifies only
  by size; richer perception (texture, shape, ripeness model) would improve
  selectivity.

---

## Team

| Member | Responsibility |
|--------|---------------|
| **Opoku Joel** | Kinematics, trajectory planning, harvest pipeline |
| **Mensah Kofi** | RGB-D perception and fruit localisation |
| **Wamyil Joseph** | Closed-loop control |
| **Abdullahi Farouk** | Rover control and end-effector functions |
