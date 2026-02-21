# Drone Navigation with SLAM – ROS 2 Jazzy / Gazebo Harmonic

> **FYP Project** – Autonomous drone navigation in simulation using SLAM, Nav2, and three selectable path-planning algorithms.

---

## Table of Contents

1. [Overview](#overview)
2. [Repository Structure](#repository-structure)
3. [Architecture](#architecture)
4. [Prerequisites](#prerequisites)
5. [Installation](#installation)
6. [Quick Start](#quick-start)
7. [Phase 1 – SLAM Exploration](#phase-1--slam-exploration)
8. [Switching Maps](#switching-maps)
9. [Phase 2 – Autonomous Navigation](#phase-2--autonomous-navigation)
10. [Navigation Algorithms](#navigation-algorithms)
11. [Keyboard Teleoperation](#keyboard-teleoperation)
12. [Package Reference](#package-reference)
13. [Troubleshooting](#troubleshooting)

---

## Overview

This project builds a complete ROS 2 drone-navigation pipeline:

| Step | What happens |
|------|-------------|
| **1** | A quadrotor drone is spawned in a Gazebo Harmonic world. |
| **2** | **slam_toolbox** runs on the drone's 360 ° lidar to build a 2-D occupancy map while you drive the drone with the keyboard. |
| **3** | The finished map is saved to disk. |
| **4** | Nav2 loads the saved map and lets you command the drone from **Point A → Point B** using one of three path-planning algorithms. |

---

## Repository Structure

```
src/
├── drone_description/   # Quadrotor URDF/XACRO, RViz configs
├── drone_gazebo/        # Gazebo Harmonic worlds (3) + spawn launch
├── drone_slam/          # slam_toolbox config + launch
├── drone_navigation/    # Nav2 configs for 3 algorithms + goal scripts
├── drone_teleop/        # Keyboard teleoperation node
└── drone_bringup/       # Top-level bringup launches + bridge config
```

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         Gazebo Harmonic                          │
│  ┌──────────────┐   /scan   ┌────────────────────────────────┐  │
│  │ Drone (URDF) ├──────────>│       ros_gz_bridge            │  │
│  │  + lidar     │  /odom    │  Gz <-> ROS 2 topic conversion │  │
│  │  + IMU       ├──────────>└────────────────────────────────┘  │
│  │  + diff-drive│<── /cmd_vel                                    │
│  └──────────────┘                                                │
└──────────────────────────────────────────────────────────────────┘
         | /scan, /odom, /tf
         v
┌──────────────────────────────────────────────────────────────────┐
│               ROS 2 Navigation Stack                             │
│  slam_toolbox --> /map --> nav2_costmap --> planner_server -->   │
│                                         (Dijkstra / A* / Smac)  │
│                               controller_server --> /cmd_vel     │
│  bt_navigator <------------ action: NavigateToPose               │
└──────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| Ubuntu      | 24.04 LTS (Noble) – also works on WSL 2 |
| ROS 2       | **Jazzy** |
| Gazebo      | **Harmonic** (gz-sim 8) |

### Install ROS 2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation.html

### Install Gazebo Harmonic + ros_gz

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge
```

### Install Nav2 and slam_toolbox

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-navfn-planner \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-nav2-smoother \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

---

## Installation

```bash
# 1. Clone this repository
git clone https://github.com/desms11/fyp_drone_nav_v1.git
cd fyp_drone_nav_v1

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install

# 5. Source the workspace overlay
source install/setup.bash
```

> **WSL 2 tip**: Add `export LIBGL_ALWAYS_SOFTWARE=1` to your `~/.bashrc` if Gazebo fails to open a window.

---

## Quick Start

### Terminal 1 – Phase 1: SLAM

```bash
source install/setup.bash
ros2 launch drone_bringup slam_bringup.launch.py world:=simple_room
```

### Terminal 2 – Keyboard Teleoperation

```bash
source install/setup.bash
ros2 run drone_teleop teleop_keyboard.py
```

Drive the drone around until the entire map is covered in RViz.

### Terminal 3 – Save the map

```bash
source install/setup.bash
mkdir -p ~/maps
ros2 launch drone_slam map_saver.launch.py map_name:=simple_room save_dir:=$HOME/maps
```

### Terminal 4 – Phase 2: Navigation

```bash
source install/setup.bash
ros2 launch drone_bringup nav_bringup.launch.py \
    map:=$HOME/maps/simple_room.yaml \
    algorithm:=astar \
    world:=simple_room
```

### Terminal 5 – Set navigation goal (interactive)

```bash
source install/setup.bash
ros2 run drone_navigation set_nav_goal.py
```

Or via CLI (non-interactive):

```bash
ros2 run drone_navigation navigate_with_algorithm.py \
    --start 0 0 --goal 4 3 --algorithm smac
```

---

## Phase 1 – SLAM Exploration

SLAM uses **slam_toolbox** in online-async mapping mode.

```
slam_toolbox
  Input  : /scan   (LaserScan, 360 degrees @ 10 Hz)
  Input  : /odom   (Odometry from Gazebo diff-drive plugin)
  Output : /map    (OccupancyGrid, published continuously)
```

**Best practices:**

- Drive slowly (< 0.3 m/s) near obstacles.
- Make sure every corridor is visited at least once.
- Revisit areas to trigger loop-closure corrections.
- Watch the map in RViz – walls should be solid lines.

**Save the map:**

```bash
ros2 launch drone_slam map_saver.launch.py \
    map_name:=my_map \
    save_dir:=$HOME/maps
```

This creates `~/maps/my_map.yaml` and `~/maps/my_map.pgm`.

---

## Switching Maps

Three Gazebo worlds are provided:

| World name    | Description                               | Size   |
|---------------|-------------------------------------------|--------|
| `simple_room` | 12 x 12 m room with boxes and cylinders   | Small  |
| `maze`        | 18 x 18 m corridor maze                   | Medium |
| `office`      | 16 x 16 m open-plan office               | Medium |

### How to switch the world

**At launch time** (recommended):

```bash
# SLAM phase with maze world
ros2 launch drone_bringup slam_bringup.launch.py world:=maze

# Navigation phase with maze world
ros2 launch drone_bringup nav_bringup.launch.py \
    world:=maze \
    map:=$HOME/maps/maze_map.yaml \
    algorithm:=astar
```

**Add your own world:**

1. Create an SDF file in `src/drone_gazebo/worlds/my_world.sdf`
   (copy `simple_room.sdf` as a template).
2. Rebuild: `colcon build --packages-select drone_gazebo --symlink-install`
3. Launch: `ros2 launch drone_bringup slam_bringup.launch.py world:=my_world`

---

## Phase 2 – Autonomous Navigation

### Setting Point A (start) and Point B (goal)

**Option 1 – Interactive script:**

```bash
ros2 run drone_navigation set_nav_goal.py
```

Follow the prompts:
```
[1/3] Set the START position (Point A):
  Start X [m]: 0
  Start Y [m]: 0
[2/3] Set the GOAL position (Point B):
  Goal X [m]: 5
  Goal Y [m]: 3
  Set goal orientation? [y/N]: n
[3/3] Choose a navigation algorithm:
  1) Dijkstra
  2) A*
  3) Smac Hybrid-A*
Your choice: 2
```

**Option 2 – CLI (non-interactive):**

```bash
ros2 run drone_navigation navigate_with_algorithm.py \
    --start 0 0 \
    --goal 5 3 \
    --algorithm astar \
    --goal-yaw 1.57
```

**Option 3 – RViz 2D Nav Goal tool:**

1. In RViz, click the **"2D Goal Pose"** button in the toolbar.
2. Click and drag on the map to set the goal position and orientation.
3. Nav2 automatically uses the currently configured algorithm.

To set the initial pose estimate, use the **"2D Pose Estimate"** button.

---

## Navigation Algorithms

Three path-planning algorithms are available, all integrated into Nav2.

### Algorithm 1 – Dijkstra (NavFn)

```bash
ros2 launch drone_navigation navigation.launch.py \
    map:=$HOME/maps/my_map.yaml algorithm:=dijkstra
```

| Property | Details |
|----------|---------|
| Plugin   | `nav2_navfn_planner/NavfnPlanner` (use_astar=false) |
| Strategy | Uniform-cost BFS; explores all directions equally |
| Path quality | Guarantees the shortest path |
| Speed    | Slower on large maps (exhaustive search) |
| Best for | Small maps where optimality matters |

### Algorithm 2 – A* (NavFn)

```bash
ros2 launch drone_navigation navigation.launch.py \
    map:=$HOME/maps/my_map.yaml algorithm:=astar
```

| Property | Details |
|----------|---------|
| Plugin   | `nav2_navfn_planner/NavfnPlanner` (use_astar=true) |
| Strategy | Heuristic-guided search (Euclidean distance to goal) |
| Path quality | Optimal with admissible heuristic |
| Speed    | Significantly faster than Dijkstra on large maps |
| Best for | General-purpose navigation; good balance of speed/quality |

### Algorithm 3 – Smac Hybrid-A*

```bash
ros2 launch drone_navigation navigation.launch.py \
    map:=$HOME/maps/my_map.yaml algorithm:=smac
```

| Property | Details |
|----------|---------|
| Plugin   | `nav2_smac_planner/SmacPlannerHybrid` |
| Strategy | Hybrid A* with kinematic constraints (Dubin curves) |
| Path quality | Smooth, physically-feasible paths |
| Speed    | Fast with analytic expansion |
| Best for | Narrow corridors, smooth trajectories, larger maps |

### Switching algorithms at runtime

Simply restart the navigation launch with a different `algorithm:=` value.
No map rebuild is required.

```bash
# Switch from A* to Smac
ros2 launch drone_navigation navigation.launch.py \
    map:=$HOME/maps/my_map.yaml algorithm:=smac
```

---

## Keyboard Teleoperation

Run in a separate terminal:

```bash
ros2 run drone_teleop teleop_keyboard.py
```

### Controls

```
w / s   : forward / backward  (linear X)
a / d   : rotate left / right (angular Z / yaw)
u / j   : strafe left / right (linear Y)
t / g   : ascend / descend    (linear Z)
SPACE   : emergency stop (zero all velocities)
q       : quit
```

Each key press increments the velocity by a configurable step.

**Optional parameters:**

```bash
ros2 run drone_teleop teleop_keyboard.py \
    --ros-args \
    -p max_linear_vel:=1.0 \
    -p max_angular_vel:=2.0 \
    -p linear_step:=0.1 \
    -p cmd_vel_topic:=/cmd_vel
```

---

## Package Reference

### `drone_description`

| File | Purpose |
|------|---------|
| `urdf/drone.urdf.xacro` | Quadrotor robot model (body, 4 rotors, lidar, IMU) |
| `rviz/drone.rviz` | RViz config for model visualisation |
| `rviz/navigation.rviz` | RViz config for navigation (map, costmap, path) |
| `launch/display.launch.py` | Stand-alone URDF viewer |

### `drone_gazebo`

| File | Purpose |
|------|---------|
| `worlds/simple_room.sdf` | 12 x 12 m room |
| `worlds/maze.sdf` | 18 x 18 m maze |
| `worlds/office.sdf` | 16 x 16 m office |
| `launch/gazebo.launch.py` | Launch Gazebo + spawn drone |

### `drone_slam`

| File | Purpose |
|------|---------|
| `config/slam_toolbox.yaml` | slam_toolbox parameters |
| `launch/slam.launch.py` | Start SLAM node |
| `launch/map_saver.launch.py` | Save map to disk |

### `drone_navigation`

| File | Purpose |
|------|---------|
| `config/nav2_dijkstra_params.yaml` | Nav2 params – Dijkstra |
| `config/nav2_astar_params.yaml`    | Nav2 params – A* |
| `config/nav2_smac_params.yaml`     | Nav2 params – Smac Hybrid-A* |
| `launch/navigation.launch.py` | Start Nav2 with chosen algorithm |
| `scripts/set_nav_goal.py` | Interactive goal setter |
| `scripts/navigate_with_algorithm.py` | CLI goal sender |

### `drone_teleop`

| File | Purpose |
|------|---------|
| `scripts/teleop_keyboard.py` | Keyboard teleoperation node |
| `launch/teleop_keyboard.launch.py` | Launch teleop in new terminal |

### `drone_bringup`

| File | Purpose |
|------|---------|
| `config/ros_gz_bridge.yaml` | Gazebo <-> ROS 2 topic bridge config |
| `launch/slam_bringup.launch.py` | Phase 1: Gazebo + SLAM |
| `launch/nav_bringup.launch.py` | Phase 2: Gazebo + Nav2 |

---

## Troubleshooting

### Gazebo does not open (WSL)

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0      # or DISPLAY=:1 on some WSL setups
```

### `transform_timeout` errors in Nav2

Increase `transform_timeout` in the costmap params, or ensure the
`/clock` bridge is running (included in `gazebo.launch.py`).

### Drone drifts with no `cmd_vel`

This is normal – the diff-drive plugin applies no damping. Press SPACE
in teleop to zero velocities.

### SLAM map has holes

- Drive slower.
- Revisit unexplored areas.
- Increase `scan_buffer_size` in `slam_toolbox.yaml`.

### Nav2 says "No valid path found"

- The goal may be inside an obstacle on the costmap.
- Try a different goal position.
- Increase `inflation_radius` slightly (0.55 -> 0.45).

### Rebuild a single package

```bash
colcon build --packages-select drone_description --symlink-install
source install/setup.bash
```

---

## References

- [Bitcraze Crazyflie Simulation](https://github.com/bitcraze/crazyflie-simulation)
- [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2)
- [ros_gz_crazyflie](https://github.com/knmcguire/ros_gz_crazyflie)
- [crazyflie_ros2_multiranger – nav-slam branch](https://github.com/knmcguire/crazyflie_ros2_multiranger/tree/add-nav-slam)
- [ROS 2 Drone Navigation in Gazebo (anson10)](https://github.com/anson10/ROS-2-Drone-Navigation-in-Gazebo)
- [Nav2 Documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)

---

*Built for Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic.*
