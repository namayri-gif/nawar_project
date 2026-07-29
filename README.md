# AI Worker Simulation: Human-Aware Warehouse Robot

Final internship project using **ROS 2 Jazzy, Gazebo, Nav2, SLAM Toolbox, OpenCV, YOLOv4-Tiny, MoveIt 2 configuration, and ros2_control**.

## 1. Project Idea

The project extends the ETGAH `ffw_sh5` warehouse robot so it can navigate to a goal and react when it detects a person.

The final behaviour is:

```text
Receive a navigation goal
        ↓
Save the original goal
        ↓
Send the goal to Nav2
        ↓
Move through the warehouse
        ↓
Detect a person with the RGB camera
        ↓
Measure the distance using the depth camera
        ↓
Cancel the active Nav2 goal
        ↓
Confirm that the robot has stopped
        ↓
Wave with the right arm
        ↓
Return the arm to its home position
        ↓
Resend the saved goal
        ↓
Continue to the original destination
```

The robot does **not** approach or rotate toward the person. It pauses, waves, and continues its original task.

This README explains what was changed, why the changes were needed, and how the full project works.

---

## 2. Repository Structure

```text
nawar_project/
├── README.md
└── src/
    └── ai-worker-sim/
        ├── ffw_bringup/
        ├── ffw_description/
        ├── ffw_moveit_config/
        ├── ffw_navigation/
        ├── ffw_swerve_drive_controller/
        ├── ffw_teleop/
        ├── human_detector/
        ├── library_world/
        └── warehouse_worlds/
```

| Package | Purpose |
|---|---|
| `ffw_bringup` | Starts Gazebo, spawns the robot, starts controllers, bridges topics, and merges the two lidars. |
| `ffw_description` | Contains the robot URDF/Xacro, joints, links, sensors, and Gazebo plugins. |
| `ffw_navigation` | Contains SLAM, the saved map, AMCL, Nav2 parameters, launch files, and RViz configuration. |
| `ffw_moveit_config` | Contains the SRDF, planning groups, named poses, controller mapping, `moveit_cpp.yaml`, and joint limits. |
| `human_detector` | New package containing person detection and the cancel-wave-resume interaction. |
| `ffw_swerve_drive_controller` | Converts `/cmd_vel` into commands for the robot's swerve-drive base. |
| `ffw_teleop` | Allows manual driving during mapping and testing. |
| `warehouse_worlds` | Contains the warehouse worlds and Gazebo models. |

The original ETGAH workspace provided the robot model and basic simulation. My work focused on enabling the camera, configuring navigation, preparing the arm configuration, creating human detection, and integrating the complete interaction sequence.

---

# What I Changed

## 3. Enabled the ZED Camera

The human detector needs an RGB image and depth data. The required ZED camera section therefore had to be enabled in:

```text
src/ai-worker-sim/ffw_description/gazebo/
ffw_sh5_rev1_follower/ffw_sh5_follower.gazebo.xacro
```

The camera provides:

```text
/zedm/image
/zedm/depth/image_raw
/zedm/camera_info
```

Each topic has a different role:

- `/zedm/image` is used by YOLO to detect people.
- `/zedm/depth/image_raw` gives the distance at image locations.
- `/zedm/camera_info` provides the camera intrinsics needed to calculate a 3D point.

Gazebo topics also need to be converted into ROS 2 messages. The required bridges are defined in:

```text
src/ai-worker-sim/ffw_bringup/config/common/gz_bridge.yaml
```

### Check the camera

After launching Gazebo:

```bash
ros2 topic list | grep zedm
ros2 topic hz /zedm/image
ros2 topic hz /zedm/depth/image_raw
```

---

## 4. Updated the Warehouse World

The storage warehouse was updated to the latest ETGAH version.

The update included:

- removing support for the moving fan to improve simulation speed;
- loading the latest storage-world files;
- replacing the old warehouse logo with the new logo.

The world package is:

```text
src/ai-worker-sim/warehouse_worlds
```

The simulation launch file adds its world and model folders to `GZ_SIM_RESOURCE_PATH`, allowing Gazebo to find the warehouse models and textures.

---

## 5. Configured Mapping and Nav2

The main navigation files are:

```text
src/ai-worker-sim/ffw_navigation/config/navigation.yaml
src/ai-worker-sim/ffw_navigation/config/mapper_params_online_sync.yaml
src/ai-worker-sim/ffw_navigation/launch/navigation.launch.py
src/ai-worker-sim/ffw_navigation/maps/map.yaml
```

### Mapping

SLAM Toolbox uses:

```text
/scan + /odom + TF → occupancy-grid map
```

Important frame settings are:

```yaml
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
```

To create a new map:

```bash
ros2 launch ffw_navigation navigation.launch.py use_slam:=true
```

Drive around the warehouse until the map is complete, then save it:

```bash
cd src/ai-worker-sim/ffw_navigation/maps
ros2 run nav2_map_server map_saver_cli -f map
```

This creates `map.pgm` and `map.yaml`.

### Navigation changes

`navigation.yaml` was adjusted to improve:

- robot speed;
- turning speed;
- goal tolerance;
- costmap size and inflation;
- obstacle detection ranges;
- velocity smoothing;
- AMCL initial pose;
- planner and controller behaviour.

The navigation pipeline is:

```text
Saved map + laser scan + odometry
                ↓
              AMCL
                ↓
      Robot pose in the map
                ↓
        Nav2 global planner
                ↓
           Global path
                ↓
        Nav2 local controller
                ↓
             /cmd_vel
                ↓
       Swerve-drive controller
```

Normal navigation is launched with:

```bash
ros2 launch ffw_navigation navigation.launch.py
```

The launch file starts localisation, Nav2, and RViz. RViz starts after a short delay so the Nav2 lifecycle nodes and TF publishers have time to activate.

---

## 6. Used MoveIt to Prepare the Wave

The robot has seven right-arm joints and many hand joints. MoveIt was used during development to:

- identify the correct right-arm planning group;
- confirm the exact joint names;
- inspect valid joint limits;
- test collision-safe arm positions;
- confirm the home position;
- understand which controller executes the right-arm motion.

The right-arm planning group is:

```text
arm_r
```

It contains:

```text
arm_r_joint1
arm_r_joint2
arm_r_joint3
arm_r_joint4
arm_r_joint5
arm_r_joint6
arm_r_joint7
```

The SRDF file is:

```text
src/ai-worker-sim/ffw_moveit_config/config/ffw.srdf
```

It defines groups such as `arm_r` and `hand_r`, named states such as `home`, and disabled collision pairs.

### Why `moveit_cpp.yaml` was added

The file:

```text
src/ai-worker-sim/ffw_moveit_config/config/moveit_cpp.yaml
```

was created so MoveItCpp could initialise:

- the planning-scene monitor;
- `robot_description`;
- `/joint_states`;
- the OMPL planning pipeline;
- the planner and scaling values.

This configuration was used while testing and validating arm poses.

The final runtime wave is sent directly to the right-arm trajectory controller. This gives exact timing and avoids waiting for a new motion plan every time a person is detected.

### Increasing arm speed

The default arm motion was too slow, so the joint limits were updated in:

```text
src/ai-worker-sim/ffw_moveit_config/config/joint_limits.yaml
```

The arm joints use:

```yaml
max_velocity: 5.0
max_acceleration: 5.0
```

Increasing the limit alone does not make the arm faster. The trajectory waypoint times must also be shortened while staying inside the permitted limits.

---

# Human Detection Package

## 7. Package Structure

A new Python package called `human_detector` was created:

```text
human_detector/
├── human_detector/
│   ├── person_detector_node.py
│   └── wave_interact.py
├── launch/
│   └── human_detector.launch.py
├── models/
│   ├── coco.names
│   ├── yolov4-tiny.cfg
│   └── yolov4-tiny.weights
├── package.xml
├── setup.cfg
└── setup.py
```

The package separates perception from robot control:

```text
person_detector_node.py = detect the person and measure distance
wave_interact.py        = control navigation and the wave sequence
```

This separation makes each part easier to test.

---

## 8. How `person_detector_node.py` Works

The detector subscribes to:

```text
/zedm/image
/zedm/depth/image_raw
/zedm/camera_info
/interaction_active
```

It publishes:

```text
/person_detected
/person_distance
/person_target
/person_detection/annotated
```

The detection process is:

```text
Receive RGB image
        ↓
Convert ROS image to OpenCV
        ↓
Create YOLO input blob
        ↓
Run YOLOv4-Tiny
        ↓
Keep only the COCO "person" class
        ↓
Apply non-maximum suppression
        ↓
Require several consecutive detections
        ↓
Read depth around each person
        ↓
Select the nearest person with valid depth
        ↓
Publish distance and 3D point
        ↓
Publish the detection event
```

### Why YOLOv4-Tiny

Gazebo, Nav2, RViz, controllers, and detection run together. YOLOv4-Tiny is lighter than the full YOLOv4 model and is more suitable for real-time CPU detection in this simulation.

### Detection stability

The detector requires several consecutive frames before changing to the detected state. This prevents one uncertain frame from triggering the complete interaction.

The launch file uses values such as:

```yaml
confidence_threshold: 0.50
nms_threshold: 0.40
detection_hold_frames: 3
network_input_size: 320
```

### Distance calculation

The node reads a small depth patch around the centre of the detected person instead of trusting one pixel. Invalid values are removed and the median valid depth is used.

The camera intrinsics then convert the pixel and depth into a 3D camera-frame point:

```text
pixel position + depth + camera intrinsics = 3D point
```

The distance and target are published before `/person_detected`, so the interaction node already has a valid measurement when the event arrives.

---

## 9. How `wave_interact.py` Works

This node connects Nav2, person detection, odometry, joint feedback, and the right-arm controller.

Its responsibilities are:

1. receive a goal;
2. save a copy of the goal;
3. send the goal to Nav2;
4. detect the first person event;
5. cancel the active Nav2 goal;
6. confirm that the base stopped;
7. execute the wave;
8. confirm that the arm returned home;
9. resend the original goal.

### Goal ownership

A Nav2 goal is managed through an action client. The client that sends the goal receives the goal handle used to cancel it.

For this reason, `wave_interact.py` receives goals from:

```text
/goal_pose
/interaction_goal_pose
```

It stores the pose and sends the `NavigateToPose` action goal itself. This gives the node the correct goal handle for cancellation and resume.

### Interaction state machine

```text
idle
  ↓
sending_goal
  ↓
navigating
  ↓
cancelling_goal
  ↓
waving
  ↓
returning_home
  ↓
resuming_goal
  ↓
navigating
  ↓
succeeded
```

Simplified logic:

```python
when_goal_received(goal):
    save(goal)
    send_goal_to_nav2(goal)

when_person_detected():
    if state == NAVIGATING and interaction_not_done:
        cancel_active_goal()
        stop_base()
        wave()
        return_arm_home()
        resend_saved_goal()
```

The state machine is needed because ROS 2 callbacks are asynchronous. Without it, cancellation, waving, and resume could overlap.

Only one interaction is allowed for each goal. A person who remains visible therefore does not repeatedly trigger the wave.

---

## 10. Stopping Before Waving

A successful cancel request does not prove that the robot is physically stationary.

The node therefore:

1. publishes zero velocity commands;
2. monitors `/odom`;
3. waits until linear and angular velocity stay below the configured thresholds.

The wave starts only after the base is confirmed stationary.

This prevents the robot from moving through the warehouse while its arm is raised.

---

## 11. Wave Motion

The final wave is sent to:

```text
/arm_r_controller/follow_joint_trajectory
```

Action type:

```text
control_msgs/action/FollowJointTrajectory
```

The motion is:

```text
Current arm position
        ↓
Move arm to the side
        ↓
Raise the elbow
        ↓
Move wrist left and right twice
        ↓
Return to elbow-up position
        ↓
Return all seven arm joints to zero
```

The requested wave duration is approximately `2.80 seconds`.

The home pose is:

```python
HOME_ARM_POSITIONS = [0.0] * 7
```

The node reads `/joint_states` and confirms that the arm is home before navigation resumes. This is safer than depending only on the controller's action-result message.

---

## 12. Launch File

The file:

```text
src/ai-worker-sim/human_detector/launch/human_detector.launch.py
```

starts:

```text
wave_interact
person_detector_node
```

`wave_interact` starts first. The detector starts after a short delay so the interaction node can create its subscriptions and action clients.

The launch file also loads:

- YOLO model paths;
- RGB, depth, and camera-info topics;
- detection thresholds;
- stop thresholds;
- arm timeouts;
- the right-arm controller action name.

---

# ROS 2 Communication

## 13. Main Topics

| Topic | Type | Purpose |
|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Goal from RViz. |
| `/interaction_goal_pose` | `geometry_msgs/PoseStamped` | Alternative goal input. |
| `/zedm/image` | `sensor_msgs/Image` | RGB input for YOLO. |
| `/zedm/depth/image_raw` | `sensor_msgs/Image` | Depth input. |
| `/zedm/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics. |
| `/person_detected` | `std_msgs/Bool` | Person-detection event. |
| `/person_distance` | `std_msgs/Float32` | Distance in metres. |
| `/person_target` | `geometry_msgs/PointStamped` | 3D camera-frame point. |
| `/person_detection/annotated` | `sensor_msgs/Image` | Detection preview. |
| `/interaction_active` | `std_msgs/Bool` | Shows that an interaction is running. |
| `/odom` | `nav_msgs/Odometry` | Confirms that the base stopped. |
| `/joint_states` | `sensor_msgs/JointState` | Confirms arm movement and home position. |
| `/cmd_vel` | `geometry_msgs/Twist` | Mobile-base velocity command. |
| `/wave_command` | `std_msgs/Bool` | Manual wave test. |

## 14. Main Actions

```text
/navigate_to_pose
nav2_msgs/action/NavigateToPose
```

This sends, cancels, and monitors the navigation goal.

```text
/arm_r_controller/follow_joint_trajectory
control_msgs/action/FollowJointTrajectory
```

This executes the seven-joint arm trajectory.

The custom package does not create a custom service. Topics are used for sensor data and events, while actions are used for long-running operations that require cancellation and a final result.

## 15. TF

The main transform chain is:

```text
map → odom → base_link → sensor and arm links
```

- `map → odom` comes from AMCL or SLAM Toolbox.
- `odom → base_link` comes from odometry.
- robot link transforms come from `robot_state_publisher` and the URDF.

Nav2 needs a valid `map → base_link` transform. The detector publishes the person target in the camera optical frame, but the current behaviour uses it only for distance reporting, not for approaching the person.

---

# Installation and Running

## 16. Clone and Build

```bash
git clone https://github.com/namayri-gif/nawar_project.git
cd nawar_project
```

Source ROS 2 and install dependencies:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Common required packages include:

```bash
sudo apt install -y \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-realsense2-description \
  ros-jazzy-dual-laser-merger \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-numpy
```

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

After changing code or configuration:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 17. Run the Complete Project

Use three terminals.

### Terminal 1: Gazebo and robot

```bash
cd ~/nawar_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ffw_bringup ffw_sh5_warehouse_storage_launch.launch.py
```

Wait until Gazebo loads, the robot appears, and the controllers start.

### Terminal 2: Nav2

```bash
cd ~/nawar_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ffw_navigation navigation.launch.py
```

Wait until Nav2 becomes active and RViz opens.

### Terminal 3: detection and interaction

```bash
cd ~/nawar_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch human_detector human_detector.launch.py
```

Expected startup message:

```text
Ready: receive goal -> navigate -> detect -> cancel and stop -> wave -> arm home -> resume the saved goal.
```

---

## 18. Send a Goal

### RViz

Use the **2D Goal Pose** tool. RViz publishes `/goal_pose`, which is received and owned by `wave_interact.py`.

### Terminal

```bash
ros2 topic pub --once /interaction_goal_pose \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {frame_id: 'map'},
    pose: {
      position: {x: -3.97576, y: 0.239322, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.998357, w: 0.0573035}
    }
  }"
```

The robot should move toward the goal. When a person is detected, it should stop, wave, return home, and continue toward the same destination.

---

# Testing

## 19. Test Each Part Separately

### Controllers

```bash
ros2 control list_controllers
```

Check that the swerve, right-arm, and joint-state controllers are active.

### Camera

```bash
ros2 topic hz /zedm/image
ros2 topic hz /zedm/depth/image_raw
ros2 topic echo /zedm/camera_info --once
```

### Laser and TF

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
```

### Person detection

```bash
ros2 topic echo /person_detected
ros2 topic echo /person_distance
ros2 topic echo /person_target
```

### Standalone wave

```bash
ros2 topic pub --once /wave_command std_msgs/msg/Bool "{data: true}"
```

Expected behaviour:

```text
Base stays still → arm rises → wrist waves twice → arm returns to zero
```

### Complete sequence

Expected terminal output:

```text
Sending original goal
Nav2 accepted the original goal
Person detected at distance X.XX m
Cancelling the active navigation goal immediately
Nav2 accepted cancellation; stopping the base before waving
Base is confirmed stationary
Sending maximum-speed wave trajectory
Arm controller accepted wave trajectory
Wave completed; arm reached home
Sending resumed original goal
Nav2 accepted the resumed original goal
Navigation goal succeeded
```

---

# Common Problems

## 20. Camera Topics Are Missing

Check that:

- the ZED camera section is enabled;
- its bridges exist in `gz_bridge.yaml`;
- the workspace was rebuilt;
- Gazebo was completely restarted.

```bash
ros2 topic list | grep zedm
```

## 21. Robot Detects but Does Not Stop

The goal must pass through:

```text
/goal_pose
```

or:

```text
/interaction_goal_pose
```

This allows `wave_interact.py` to own the Nav2 goal handle.

Check:

```bash
ros2 action info /navigate_to_pose
ros2 topic echo /odom
```

## 22. Robot Stops but Does Not Wave

Check:

```bash
ros2 control list_controllers
ros2 action info /arm_r_controller/follow_joint_trajectory
ros2 topic echo /joint_states
```

The right-arm action server and all seven right-arm joint states must exist.

## 23. Arm Moves Too Slowly

Check:

```text
ffw_moveit_config/config/joint_limits.yaml
```

Also check the `time_from_start` values in `wave_interact.py`. Larger times create slower movement.

After changing robot or controller limits, rebuild and restart Gazebo.

## 24. Navigation Does Not Resume

The node will not resume if:

- the original goal was not saved;
- cancellation failed;
- the arm did not return home;
- Nav2 rejected the resumed goal.

This is intentional: navigation must not restart while the arm is raised.

## 25. TF Errors

Check:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

The frame names must match in the URDF, SLAM, AMCL, Nav2, and sensor messages.

---

# 26. Final Summary

The final project joins perception, navigation, and manipulation into one ROS 2 system:

```text
Gazebo simulates the warehouse, robot, lidar, RGB camera, and depth camera.

ffw_bringup starts the simulator, controllers, bridges, and robot model.

ffw_navigation uses the saved map, laser scan, odometry, TF, AMCL, and Nav2 to move the robot.

person_detector_node.py runs YOLOv4-Tiny, detects the nearest person, and measures distance from depth data.

wave_interact.py owns the Nav2 goal, cancels it, confirms that the base stopped, sends the right-arm trajectory, confirms that the arm returned home, and resends the original goal.
```

The final behaviour is:

```text
Receive goal → navigate → detect person → print distance once
→ cancel goal → stop base → wave → arm home → resume original goal
```

The main lesson is that this is an integration project. Detection, Nav2, controllers, actions, topics, odometry, joint feedback, and safety checks must all agree before the system moves to the next step.

---


## Author

**Nawar Amayri**  
Electrical Engineering Internship Project
