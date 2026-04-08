# 🌿 Greenhouse Robotic Arm Harvesting System

A vision-guided robotic manipulator system designed for automated crop harvesting 
in greenhouse environments. This project integrates RGB-D based fruit detection, 
3D localization, quintic trajectory planning, and adaptive grasping within a 
unified ROS2/Gazebo simulation framework.

---

## 📌 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Modules](#modules)
- [Hardware Setup](#hardware-setup)
- [Simulation & Evaluation](#simulation--evaluation)
- [Expected Results](#expected-results)
- [Team](#team)
- [References](#references)

---

## Overview

Greenhouse farming relies heavily on manual labor for critical tasks like 
harvesting, sorting, and pruning. This project addresses these inefficiencies 
by designing and simulating an integrated robotic arm system capable of:

- Detecting and localizing fruits in 3D space using an RGB-D camera
- Planning smooth, collision-free trajectories
- Grasping and cutting fruit without causing damage
- Sorting harvested produce by color and size into appropriate containers

The system is validated entirely in simulation before any potential physical 
deployment.

---

## System Architecture

The system is built on **ROS2** and simulated in **Gazebo**. It consists of 
five tightly integrated modules that communicate via ROS2 topics:

```
RGB-D Camera → [detected_fruit] → Localization → [fruit_position] → Trajectory Planning
                                                                          ↓
                                                                  Closed-loop Servoing
                                                                          ↓
                                                                Grasping & Cutting
```

---

## Modules

### 1. 🎯 Vision-Based Perception
- Uses an **RGB-D camera** to detect fruits and capture depth information
- Depth data improves reliability in cluttered foliage environments
- 3D fruit position is computed and transformed to the robot's base frame:

```
P_base = T_camera->base * P_camera
```

### 2. 📍 3D Localization
- Subscribes to the `detected_fruit` topic
- Estimates fruit position and distance using RGB-D depth data
- Publishes 3D coordinates to the `fruit_position` topic

### 3. 🛤️ Trajectory Planning & Optimization
- Uses **forward kinematics (FK)** to determine current end-effector pose
- Uses **inverse kinematics (IK)** to compute required joint angles to reach the fruit
- Generates smooth joint trajectories using **quintic polynomial interpolation**:

```
y(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

This ensures smooth position, velocity, and acceleration profiles, minimizing 
abrupt motion.

### 4. 🔄 Closed-Loop Motion Control
- Receives required joint angles from the trajectory planning node
- Drives motors to the target configuration
- Sends current joint angles back for FK feedback and error correction

### 5. ✋ Adaptive Grasping, Cutting & Sorting
- A **mechanical gripper with a cutter** stabilizes and detaches the fruit at the peduncle
- A virtual **force sensor** at the end-effector prevents crop damage
- Visual feedback continuously corrects positional errors during approach
- Harvested fruits are **sorted by color and size** and placed into appropriate containers

---

## Hardware Setup (Simulation)

| Component | Description |
|-----------|-------------|
| Robotic Arm | 6-DOF arm mounted on a mobile ground rover |
| Rover | Wheeled mobile robot that navigates between greenhouse rows |
| Camera | RGB-D camera for real-time fruit detection and depth sensing |
| End-Effector | Mechanical gripper + cutter |
| Force Sensor | Virtual sensor to regulate grasp force |
| Software | ROS2 + Gazebo |
| Robot Description | URDF with collision geometry |
| Environment | Gazebo greenhouse model with plant rows and randomly spawned fruit |

> The robotic arm is mounted on a wheeled mobile rover that moves along 
greenhouse rows on the ground, as shown in the simulation screenshot below.

![Gazebo Simulation](simulation_preview.jpg)

---

## Simulation & Evaluation

The full pipeline is tested in Gazebo. Performance is tracked using:

- ✅ **Harvest success rate**
- ⏱️ **Average cycle time per fruit**
- 💥 **Collision frequency**
- 📈 **Trajectory smoothness**
- 🎯 **Fruit detection and localization accuracy**
- 🔵 **Sorting accuracy by color and size**

---

## Expected Results

| Metric | Target |
|--------|--------|
| Harvest success rate | > 80% under normal visual conditions |
| Cycle time per fruit | 10 – 20 seconds |
| Trajectory | Smooth, no abrupt speed changes |
| Self-correction | Minor positional errors corrected during approach |
| Sorting | Correct classification by color and size |

---

## Team

| Member | Responsibility |
|--------|---------------|
| **Opoku Joel** | Kinematics and trajectory planning |
| **Mensah Kofi** | RGB-D perception and fruit localization |
| **Wamyil Joseph** | Closed-loop motor control |
| **Abdullahi Farouk** | Rover control and end-effector functions |

---

## References

1. Yudha et al. — *Arm Robot Manipulator Design and Control for Trajectory Tracking: A Review* (2018)
2. Qian et al. — *Manipulation Task Simulation using ROS and Gazebo* (2014)
3. Xu et al. — *Optimal Trajectory Planning for Manipulators with Efficiency and Smoothness Constraint* (2023)
4. Romero et al. — *Trajectory Planning for Robotic Manipulators in Automated Palletizing: A Comprehensive Review* (2025)
5. Dai et al. — *A Review of Spatial Robotic Arm Trajectory Planning* (2022)
6. Van Henten et al. — *An Autonomous Robot for Harvesting Cucumbers in Greenhouses* (2002)
7. Ahmed et al. — *Cucumber Picking Robots: Technological Progress, Challenges, and Future Directions* (2026)
8. Aksoy et al. — *Real-Time Vision-Based Robotic Arm Controller Using ROS and Gazebo*
