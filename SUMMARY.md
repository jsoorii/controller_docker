# Project Summary: UR5e MuJoCo-Meshcat Simulation

## Overview

This project is a **UR5e collaborative robot simulator** that combines:
- **MuJoCo** (DeepMind's physics engine) for accurate robot dynamics
- **Meshcat** (web-based 3D visualization via Three.js/ZMQ) for browser rendering
- **ROS2 Humble** as the underlying middleware framework (Docker-based)

The goal is to simulate a Universal Robots UR5e 6-DOF robotic arm, visualize it in a web browser, and lay the groundwork for full ROS2 controller integration — all without physical hardware.

---

## Directory Structure

```
heroi/
├── Dockerfile                        # ROS2 Humble + MuJoCo container definition
├── docker-compose.yaml               # Container orchestration (port 7000, host network)
├── ros_entrypoint.sh                 # Sources ROS2 environment on container start
├── .gitignore                        # Excludes __pycache__, .venv, build/, log/, etc.
├── README.md                         # Korean-language project state report
└── my_ur5e_controller/
    ├── ur5e_rt_controller.py         # Real-time PD controller (state machine)
    ├── view_ur5e.py                  # Standalone mesh visualization script
    └── main_test.py                  # Main entry point (controller + visualizer)
```

> **Container-only** (cloned during `docker build`):
> `/ros2_ws/src/mujoco_menagerie/universal_robots_ur5e/` — UR5e URDF, XML, and `.obj` meshes

---

## Technology Stack

| Layer | Technology |
|---|---|
| Language | Python 3.10+ |
| Physics | MuJoCo (DeepMind) |
| Visualization | Meshcat (ZMQ + Three.js) |
| Middleware | ROS2 Humble |
| Container | Docker + Docker Compose |
| Math | NumPy, transforms3d |

---

## Key Components

### 1. `ur5e_rt_controller.py` — Real-Time Controller

A **5-state state machine** running at 500 Hz (2 ms timestep):

```
WAIT_STABLE → INIT_CONTROL → CHECK_MOTOR → INIT_POSITION → RUN_CONTROL
```

- Implements **PD control**: `τ = Kp × (q_desired − q_actual) + Kd × (0 − dq_actual)`
  - Kp = 500.0, Kd = 50.0
- Detects and logs timing misses (real-time performance monitoring)
- Prevents sudden joint jerks at startup via state sequencing
- Operates on shared MuJoCo `model`/`data` objects (thread-safe by design)

### 2. `view_ur5e.py` — Mesh Visualization

Standalone script for loading and rendering the robot:

- Reads UR5e URDF/XML from MuJoCo Menagerie
- Extracts mesh geometry (vertices + faces) from MuJoCo's internal arrays
- Registers `TriangularMeshGeometry` objects in Meshcat with RGBA materials
- Updates transforms (`xpos`, `xmat`) per simulation step
- Includes a test: sinusoidal shoulder joint oscillation

### 3. `main_test.py` — Main Application

Orchestrates control and visualization together:

- Loads MuJoCo model, creates Meshcat visualizer (auto-starts server on port 7000)
- Spawns controller in a **daemon thread** at 500 Hz
- Main thread runs visualization loop at ~50 Hz (20 ms sleep)
- Sends sinusoidal joint targets to the controller during `RUN_CONTROL` phase
- Monitors for instability (velocity > 50 rad/s triggers safety stop)

**Threading model:**
```
Main Thread (50 Hz)       Controller Thread (500 Hz)
  ├─ Update Meshcat          ├─ Read MuJoCo state
  ├─ Log state machine        ├─ Compute PD torques
  └─ Send joint commands      └─ Apply to MuJoCo
```

---

## Docker Infrastructure

### `Dockerfile`
- Base: `osrf/ros:humble-desktop`
- Installs: `mujoco`, `meshcat`, `numpy`, `transforms3d`, headless OpenGL libs
- Clones `mujoco_menagerie` (DeepMind robot model repository) at build time

### `docker-compose.yaml`
- Container: `ur5e_mujoco_ros2`
- Mounts project root → `/ros2_ws/src/my_ur5e_controller` inside container
- Host network mode (direct access to port 7000 for Meshcat browser)
- `MUJOCO_GL=osmesa` for headless (no display server) rendering

### `ros_entrypoint.sh`
- Sources `/opt/ros/humble/setup.bash`
- Conditionally sources built workspace if present
- Executes passed commands in the ROS2 environment

---

## Architecture Decisions

| Decision | Rationale |
|---|---|
| State machine for controller startup | Prevents sudden movements; enables safe initialization |
| Separate 500 Hz control thread | Decouples real-time control from slower visualization |
| MuJoCo Menagerie for robot model | Official DeepMind-maintained URDF/mesh repository |
| Meshcat over Rviz | Web-based, no display server needed on headless remote server |
| Host network mode in Docker | Simplifies Meshcat port access from Mac browser via SSH tunnel |
| PD control (no integral term) | Sufficient for demo; avoids steady-state error accumulation complexity |

---

## Current Status

**Working:**
- MuJoCo physics simulation of UR5e
- Real-time Meshcat web visualization (port 7000)
- PD joint position control with state machine
- Threaded control + visualization architecture
- Dockerized, reproducible environment

**Not yet implemented (Next Milestones):**
1. **Joint State Publisher** — publish MuJoCo joint states as ROS2 `sensor_msgs/JointState`
2. **Robot State Publisher** — TF tree construction from URDF for ROS2 navigation/planning
3. **ROS2 Controller** — accept `JointTrajectory` commands via ROS2 topics instead of hardcoded sinusoidal input
4. **Multi-robot / model scaling** — extend to other robot models using ROS2 package structure

---

## Quick Start

```bash
# Build and launch container
docker compose build
docker compose up -d

# Run main simulation (inside container)
docker exec -it ur5e_mujoco_ros2 bash
python3 /ros2_ws/src/my_ur5e_controller/main_test.py

# View in browser (from Mac via SSH tunnel or on same machine)
# http://localhost:7000
```
