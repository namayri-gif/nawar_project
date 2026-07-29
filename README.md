# AI Worker Simulation — Warehouse Navigation & Human Interaction

**Final internship project — ROS 2 (Jazzy) / Nav2 / MoveIt / Gazebo**

This repository implements **Project 1** of the AI Worker Simulation brief: an `ffw_sh5` warehouse robot that maps its environment, navigates autonomously between goals, and reacts naturally when it encounters a person — stopping, waving, and resuming its route on its own.


---

## My contribution

The `ffw_sh5` robot stack (description, bringup, drive controller, teleop) came from the shared internship base workspace. My work on top of that base was:

- **`human_detector` package — built from scratch.** Both nodes (`person_detector_node.py` for YOLOv4-tiny person detection, and `wave_interact.py` for the Nav2 goal ownership / cancel / wave / resume state machine), the launch file wiring them together with MoveItPy, and the model files.
- **`ffw_description`** — edits to the robot's Gazebo plugins (sensor/plugin configuration) to get the camera and lidar feeding the topics this project needed.
- **`ffw_navigation`** — configured and ran the SLAM Toolbox / Nav2 setup for the warehouse world: tuned `navigation.yaml`, generated and saved the map used for localization, and validated multi-goal navigation and obstacle avoidance.
- **`ffw_moveit_config`** — configured and used the MoveIt setup for the wave motion: the `arm_r` / `hand_r` planning groups and the `ready` / `open` SRDF states that `wave_interact.py` plans against.

---

## What it does

The robot runs a full warehouse loop:

1. **Maps** the warehouse world with SLAM Toolbox and saves a reusable occupancy grid map.
2. **Navigates** autonomously to goal poses using Nav2, relying on its costmap for obstacle avoidance around shelving, pallets, and walls.
3. **Watches** the world through its onboard camera with a YOLOv4-tiny detector looking specifically for people (separate from Nav2's lidar-based obstacle avoidance).
4. **Reacts** the moment a person is detected: it cancels its current Nav2 goal, plays a MoveIt-planned wave with its right arm and hand, and then automatically resumes the original goal — no need to wait for the person to move.

```
 SLAM mapping  →  Nav2 navigation  →  person detected?  →  cancel goal  →  wave (MoveIt)  →  resume goal
```

| Stage | What happens |
|---|---|
| SLAM mapping | Builds the warehouse map with `slam_toolbox` |
| Nav2 navigation | Sends and executes navigation goals |
| Obstacle avoidance | Nav2 costmap avoids static objects (shelves, pallets, walls) |
| Human detection | Camera + YOLOv4-tiny identifies a person |
| Stop | Current Nav2 goal is cancelled |
| Wave | MoveIt executes a fixed wave trajectory on the right arm/hand |
| Resume | The original goal is automatically resent to Nav2 |

---

## Repository layout

```
project_final/
└── src/
    ├── ai-worker-sim/               # Robot stack (description, control, nav, moveit, detection)
    │   ├── ffw_description/         # URDF/xacro robot model (base + my Gazebo plugin edits)
    │   ├── ffw_bringup/              # Gazebo launch files (incl. warehouse_storage launch) — base
    │   ├── ffw_navigation/           # Nav2 + SLAM Toolbox config, launch, and saved map — configured/used by me
    │   ├── ffw_moveit_config/        # MoveIt config (arm_r / hand_r planning groups, SRDF states) — configured/used by me
    │   ├── ffw_swerve_drive_controller/  # base
    │   ├── ffw_teleop/                    # base
    │   └── human_detector/           # Person detection (YOLOv4-tiny) + wave/interaction node — built by me
    └── Gazebo_worlds/
        └── warehouse_worlds/         # Warehouse Gazebo worlds (storage, logistics, distribution)
```

The two nodes that make Phase 2 work both live in `human_detector/human_detector/`:

- **`person_detector_node.py`** — subscribes to the camera feed, runs a YOLOv4-tiny (OpenCV DNN) forward pass filtered to the COCO `person` class, debounces detections over several consecutive frames to avoid flicker, and publishes a latched `/person_detected` (`std_msgs/Bool`).
- **`wave_interact.py`** — owns the Nav2 goal handle and the MoveItPy interface. It accepts goals on `/interaction_goal_pose`, drives the full state machine (`idle → sending → navigating → cancelling → waving → resuming → succeeded`), and plans/executes the wave (open hand → arm to `ready` → 2 wave cycles → arm back to `ready`) using the `arm_r` and `hand_r` MoveIt planning groups.

---

## Requirements

- Ubuntu 24.04 + **ROS 2 Jazzy**
- Gazebo (`ros_gz_sim`)
- Nav2 and SLAM Toolbox
- MoveIt 2 (with `moveit_py`)

```bash
sudo apt update
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-realsense2-description
sudo apt install ros-jazzy-dual-laser-merger
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

Python dependencies for the detector: `opencv-python`, `numpy`, `cv_bridge` (installed via the ROS packages above, plus `python3-opencv`).

---

## Build

```bash
git clone https://github.com/namayri-gif/project_final.git
cd project_final
colcon build
source install/setup.bash
```

---
## Phase 1:

### 1. Launch the warehouse simulation

```bash
ros2 launch ffw_bringup ffw_sh5_warehouse_storage_launch.launch.py
```

This brings up Gazebo with the storage warehouse world, spawns the `ffw_sh5` robot, starts the controllers, the laser merger, and RViz.

### 2. Mapping 

To build a new map with SLAM Toolbox:

```bash
ros2 launch ffw_navigation online_sync_launch.py use_slam:=true
```

Drive the robot around the warehouse (teleop or Nav2 goals) until the map looks complete, then save it with `slam_toolbox`'s map saver (or `nav2_map_server`'s `map_saver_cli`). The map used for this submission is already saved at:

### 3. Saving the Map
```
cd ros2 _ws/src/ai-worker-sim/ffw_navigation/maps
ros2 run nav2_map_server map_saver_cli -f map2
```
### 4. Nav2 Goals

```bash
ros2 launch ffw_navigation navigation.launch.py
```

- Set Pose Estimate
- Set 2D Goal Pose

## Phase 2

### 1. Human detection + wave interaction

```bash
ros2 launch human_detector human_detector.launch.py
```

This starts `wave_interact` (MoveItPy + Nav2 client) immediately, and `person_detector_node` after a short delay so MoveIt has time to come up.

### 2. Send a navigation goal through terminal

Goals are sent through `wave_interact`'s own topic (rather than directly to Nav2) so that the node keeps ownership of the goal handle and can cleanly cancel and resume it. This is the exact goal used for the demo run:

```bash
ros2 topic pub --once /interaction_goal_pose \
geometry_msgs/msg/PoseStamped \
"{
  header: {
    frame_id: 'map'
  },
  pose: {
    position: {
      x: -4.648,
      y: -1.165,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: 0.999654,
      w: 0.026293
    }
  }
}"
```

The robot starts navigating toward that pose. If the vision pipeline detects a person along the way, it cancels the goal, waves, and automatically resumes toward `(-4.648, -1.165)` once the wave finishes.

You can also trigger the wave on its own, without any navigation, for a quick sanity check:

```bash
ros2 topic pub --once /wave_command std_msgs/msg/Bool "{data: true}"
```

---

## Interaction state machine

`wave_interact.py` tracks a single interaction state so goal sending, cancellation, waving, and resuming can never overlap:

```
idle → sending → navigating → cancelling → waving → resuming → succeeded
                     │                                              │
                     └───────────────── (goal reached directly) ────┘
```

- A new goal on `/interaction_goal_pose` is only accepted while the robot is `idle`.
- `/person_detected` only triggers a cancellation while the robot is actively `navigating`, and is latched so a continuously-visible person doesn't retrigger the sequence.
- The wave only starts after Nav2 reports the goal as **terminally cancelled** — not as soon as the cancel request is merely accepted.
- The original goal is resumed automatically once the wave sequence completes successfully; if planning or execution fails at any step, the state machine moves to `failed` instead of silently continuing.

---

## Notes on the wave

- The wave uses a fixed joint-space sequence for `arm_r` (open `hand_r` → `arm_r` to `ready` → 3 alternating left/right sweep positions → back to `ready`), planned and executed with MoveItPy.
- `arm_r` and `hand_r` are configured as separate MoveIt planning groups (`ffw_moveit_config/config/ffw.srdf`), which is what lets the hand open independently before the arm sweeps.

---

## Demos and Pictures

Mapping Video and Full Map pgm:

[scrnli_5yrzgF9Qf7FLP3.pdf](https://github.com/user-attachments/files/30356629/scrnli_5yrzgF9Qf7FLP3.pdf)



Person Loaded into Gazebo world:

[scrnli_68n6iVABcon2aL.pdf](https://github.com/user-attachments/files/30356497/scrnli_68n6iVABcon2aL.pdf)

Phase 2 launch:

Detection:

https://github.com/user-attachments/assets/82776d06-bafd-40f1-ac10-93a9935db2fc


Wave Movement:

https://github.com/user-attachments/assets/0b04ab59-027e-463e-ad46-50e922cb64da


Resuming and Reaching the Goal: 

https://github.com/user-attachments/assets/b4a88a51-02a7-4623-b53d-f4e2c341d1f5




















---
