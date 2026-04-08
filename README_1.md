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
