# TurtleBot Anomaly Detection & Surveillance System

A ROS-based security system that enables a TurtleBot to autonomously map an environment, patrol key areas, detect anomalies, and respond by collecting data such as images and positional information.

---

## Overview

Build a security system to detect anomalies (apart from buildings and boundaries), estimate their location, and send photos for review.

> **Anomalies Meaning:** Something that is unusual, unexpected, or doesn't fit a pattern; similar to "abnormality" but often used in scientific or technical contexts.

---

## 🔄 System Workflow

### Step 1: Environment Mapping

Initiate a mapping routine using TurtleBot's onboard sensors (e.g., SLAM with LiDAR or depth camera) to generate an accurate map of the operational area.

### Step 2: Define Surveillance Routine

Set up a predefined patrol or surveillance path using waypoints or navigation goals, enabling continuous monitoring of critical zones.

### Step 3: Anomalous Object Detection

Detect unexpected or unauthorized objects in the environment using:

- LiDAR-based shape deviation analysis
- Visual classification via neural networks (camera input)

### Step 4: Target Engagement and Data Collection

Upon anomaly detection:
- Navigate toward the target
- Capture detailed images
- Record GPS or relative coordinates
- Emit sound alerts or audio messages

### Step 5: Resume Patrol

After engagement and data collection, return to the previous patrol route and continue surveillance operations.

---

## Development Phases

### Phase 1: Basic Detection and Localization

- Detect object via LiDAR
- Estimate position using environment maps
- Refine location using camera data

### Phase 2: Object Classification

- Integrate a pre-trained image classification model
- Categorize detected anomalies (e.g., unknown object, misplaced item, person, etc.)

---

## 📦 Dependencies

- ROS (Robot Operating System)
- TurtleBot3 packages
- SLAM Toolbox or GMapping
- OpenCV (for image processing)
- TensorFlow / PyTorch (for neural network inference)
- RViz (for visualization)
- Python 3.x

---

## 📂 Project Structure
### Building the Package

1. **Launch the Virtual Environment:**
   - Open **Terminal 1** and run:
     ```bash
     cd ~/ros2_lecture.ws
     . 0_env.sh
     . /entrypoint.sh
     ```

2. **Build the Workspace:**
   - In **Terminal 1**, execute:
     ```bash
     colcon build --symlink-install
     ```
   - **Note:** The `--symlink-install` option allows Python script modifications without rebuilding.

### Opening the Editor (Optional)

- To view or edit files, such as `README.md`:
  1. Open Visual Studio Code:
     ```bash
     code ~/ros2_lecture.ws
     ```
  2. Alternatively, use the GUI: Open VS Code, select "Open Folder," and choose `~/ros2_lecture.ws`.
  3. In VS Code, navigate to `lecture03_pkg`, click `README.md`, and use the preview icon to view it.

---

## Connecting to TurtleBot3

Connect to the TurtleBot3 physically, power it on, synchronize time with the PC, and launch the robot's system.

### Hardware Connections

1. **Connect the Battery:**
   - Attach the battery terminals to the TurtleBot3.
   - Insert the battery into its designated slot.

2. **Connect via LAN:**
   - Use a LAN cable to connect the TurtleBot3 to your PC.

### Turning On the Robot and PC

1. **Turn On TurtleBot3:**
   - Switch the power button to ON.
   - Wait ~3 minutes for startup; the Laser Range Finder (LRF) will start rotating.

2. **Turn On the PC:**
   - Log in with:
     - **User:** `ros2`
     - **Password:** `ros1ecture`

### Synchronizing Time between PC and TurtleBot3

1. **Switch Connection Mode:**
   - In **Terminal 1**, run:
     ```bash
     turtlebots_mode
     ```
   - Enter password: `ros1ecture` (input is hidden).

2. **Check Synchronization:**
   - In **Terminal 1**, run:
     ```bash
     cd ~/ros2_lecture.ws
     ./sync_time.sh
     ```
   - If synchronization fails, repeat the `turtlebots_mode` command and retry.

### Launching the System

1. **SSH Connection:**
   - Open **Terminal 2** and connect via SSH:
     ```bash
     ssh -YC turtle@192.168.11.2
     ```
   - Enter password: `turtlebot` (input is hidden).

2. **Launch the Robot System:**
   - In **Terminal 2** (SSH session), run:
     ```bash
     ros2 launch ros2_lecture bringup.launch.py
     ```
   - **Note:** Keep **Terminal 1** running.

---

## Generating a Map using SLAM

Use SLAM to create an environmental map by launching the SLAM system, controlling the robot, and saving the map.

### Launching SLAM

1. **Ensure Connection and System Launch:**
   - Verify that the steps in "Connecting to TurtleBot3" are complete and the system is running in **Terminal 2**.

2. **Launch the Virtual Environment:**
   - Open **Terminal 3** and run:
     ```bash
     cd ~/ros2_lecture.ws
     . 0_env.sh
     . /entrypoint.sh
     ```

3. **Launch SLAM:**
   - In **Terminal 3**, execute:
     ```bash
     . 4a_turtlebot3_settings.sh
     ros2 launch turtlebot3_cartographer cartographer.launch.py
     ```
   - A successful launch will display the map forming in Rviz2.

### Controlling the Robot for Mapping

1. **Launch the Virtual Environment:**
   - Open **Terminal 4** and run:
     ```bash
     cd ~/ros2_lecture.ws
     . 0_env.sh
     . /entrypoint.sh
     ```

2. **Run the Teleop Keyboard Node:**
   - In **Terminal 4**, execute:
     ```bash
     . 4a_turtlebot3_settings.sh
     ros2 run turtlebot3_teleop teleop_keyboard
     ```
   - Control the TurtleBot3 slowly using:
     - `W`: Move forward
     - `S`: Move backward
     - `A`: Turn left
     - `D`: Turn right
     - `X`: Stop

### Saving the Map Data

1. **Launch the Virtual Environment:**
   - Open **Terminal 5** and run:
     ```bash
     cd ~/ros2_lecture.ws
     . 0_env.sh
     . /entrypoint.sh
     ```

2. **Save the Map:**
   - In **Terminal 5**, execute:
     ```bash
     . 4a_turtlebot3_settings.sh
     ros2 run nav2_map_server map_saver_cli -f ~/ros2_lecture_ws/map
     ```
   - This saves `map.pgm` and `map.yaml` in `~/ros2_lecture_ws`.

3. **Shutdown:**
   - Stop all terminals (`Terminal 1`, `2`, `3`, `4`, `5`) with `Ctrl+C` after verifying the map is saved.

---

## Gazebo Simulation
### Install simulation package
The TurtleBot3 Simulation Package requires turtlebot3 and turtlebot3_msgs packages. Without these prerequisite packages, the Simulation cannot be launched.
Please follow the PC Setup instructions if you did not install required packages and dependent packages.

```
$ cd ~/turtlebot3_ws/src/
$ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/turtlebot3_ws && colcon build --symlink-install
```

### Copy The Map
Copy the map.pgm and map.yml file

### Launch Simulation World
```
$ export TURTLEBOT3_MODEL=waffle
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Run Slam Node
```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

### Run Navigation Node
```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```
change $HOME/map.yaml, with the name of the map that you want to use.

