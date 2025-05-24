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

#### patrol code


For ROS
🛠 Step 1: Prepare the ROS 2 Workspace

If you're already inside the `turtlebot3_ws/src` directory, make sure your ROS 2 workspace has been initialized. If not, follow these steps:

### 1.1 Check ROS 2 Workspace

Make sure you're inside the `src` directory of the ROS 2 workspace:

```bash
cd ~/turtlebot3_ws/src
```

### 1.2 Create a ROS 2 Python Package

Now, let’s create a new package. This package will use `ament_python` as the build system for Python.

Run this command:

```bash
ros2 pkg create --build-type ament_python turtlebot3_drive_py
```

This will create a `turtlebot3_drive_py` folder inside `src` with the basic structure of a ROS 2 Python package.

---

📝 Step 2: Set Up Package Files

After creating the package, we will set up the necessary files inside it.

### 2.1 Enter the Package Folder

```bash
cd turtlebot3_drive_py
```

### 2.2 Create the Python File (`turtlebot3_drive.py`)

Create a Python file inside your package directory:

```bash
mkdir turtlebot3_drive_py
nano turtlebot3_drive_py/turtlebot3_drive.py
```

Paste the previously converted Python code into this file (as shared in the first answer).

Save it by pressing `Ctrl + X`, then press `Y`, and then press `Enter`.

### 2.3 Edit `setup.py`

Now open the `setup.py` file:

```bash
nano setup.py
```

Make sure it contains the following:

```python
from setuptools import setup

package_name = 'turtlebot3_drive_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Python version of TurtleBot3 Drive node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_drive = turtlebot3_drive_py.turtlebot3_drive:main'
        ],
    },
)
```

Save the changes.

### 2.4 Edit `package.xml`

Open `package.xml` to ensure required dependencies are included:

```bash
nano package.xml
```

Make sure it contains the following:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>turtlebot3_drive_py</name>
  <version>0.1.0</version>
  <description>Python version of TurtleBot3 Drive node</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf-transformations</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Save the changes.

---

🛠 Step 3: Build and Install the Package

### 3.1 Return to Main Workspace Directory

After editing the package files, return to the root of the workspace:

```bash
cd ~/turtlebot3_ws
```

### 3.2 Build the Workspace

Build the workspace using `colcon`:

```bash
colcon build --packages-select turtlebot3_drive_py
```

### 3.3 Source the Setup File

After the build is finished, source the setup file to activate the new package:

```bash
source install/setup.bash
```

---

▶️ Step 4: Run the Python Node

### 4.1 Run the Node

Now you can run the node you just created:

```bash
ros2 run turtlebot3_drive_py turtlebot3_drive
```

If everything runs correctly, you will see a message in the terminal indicating that the TurtleBot3 node has been initialized.

---

🔄 Step 5: Optional - Test in Simulation

If you are using the TurtleBot3 simulation with Gazebo, you can test the node by running the simulation and observing the robot's behavior.

Launch the TurtleBot3 simulation in Gazebo (if installed):

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Then, run the Python node you created and observe the robot’s movement in the simulation.

---

✅ Step 6: Add Dependencies (If Needed)

If you encounter errors or missing dependencies, make sure to install the necessary ROS 2 packages. For example, if `tf-transformations` is missing, you can install it with:

```bash
sudo apt-get install ros-<ros_distro>-tf-transformations
```

Replace `<ros_distro>` with your ROS 2 distribution name (e.g., `humble`, `foxy`).

---
### turtlebot3_drive.py
```
#!/usr/bin/env python3
# Copyright 2019 ROBOTIS CO., LTD.

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

DEG2RAD = math.pi / 180.0
LINEAR_VELOCITY = 0.3
ANGULAR_VELOCITY = 1.5

GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3

CENTER = 0
LEFT = 1
RIGHT = 2


class Turtlebot3Drive(Node):
    def __init__(self):
        super().__init__('turtlebot3_drive_node')

        # Init Var
        self.scan_data = [0.0, 0.0, 0.0]
        self.robot_pose = 0.0
        self.prev_robot_pose = 0.0
        self.turtlebot3_state_num = 0

        # QoS
        qos = rclpy.qos.QoSProfile(depth=10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos)

        # Timer
        self.update_timer = self.create_timer(0.01, self.update_callback)

        self.get_logger().info('Turtlebot3 simulation node has been initialised')

    def __del__(self):
        self.get_logger().info('Turtlebot3 simulation node has been terminated')

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.robot_pose = yaw

    def scan_callback(self, msg):
        scan_angle = [0, 30, 330]

        for i in range(3):
            angle = scan_angle[i]
            if math.isinf(msg.ranges[angle]):
                self.scan_data[i] = msg.range_max
            else:
                self.scan_data[i] = msg.ranges[angle]

    def update_cmd_vel(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def update_callback(self):
        escape_range = 30.0 * DEG2RAD
        check_forward_dist = 0.7
        check_side_dist = 0.6

        if self.turtlebot3_state_num == GET_TB3_DIRECTION:
            if self.scan_data[CENTER] > check_forward_dist:
                if self.scan_data[LEFT] < check_side_dist:
                    self.prev_robot_pose = self.robot_pose
                    self.turtlebot3_state_num = TB3_RIGHT_TURN
                elif self.scan_data[RIGHT] < check_side_dist:
                    self.prev_robot_pose = self.robot_pose
                    self.turtlebot3_state_num = TB3_LEFT_TURN
                else:
                    self.turtlebot3_state_num = TB3_DRIVE_FORWARD

            elif self.scan_data[CENTER] < check_forward_dist:
                self.prev_robot_pose = self.robot_pose
                self.turtlebot3_state_num = TB3_RIGHT_TURN

        elif self.turtlebot3_state_num == TB3_DRIVE_FORWARD:
            self.update_cmd_vel(LINEAR_VELOCITY, 0.0)
            self.turtlebot3_state_num = GET_TB3_DIRECTION

        elif self.turtlebot3_state_num == TB3_RIGHT_TURN:
            if abs(self.prev_robot_pose - self.robot_pose) >= escape_range:
                self.turtlebot3_state_num = GET_TB3_DIRECTION
            else:
                self.update_cmd_vel(0.0, -ANGULAR_VELOCITY)

        elif self.turtlebot3_state_num == TB3_LEFT_TURN:
            if abs(self.prev_robot_pose - self.robot_pose) >= escape_range:
                self.turtlebot3_state_num = GET_TB3_DIRECTION
            else:
                self.update_cmd_vel(0.0, ANGULAR_VELOCITY)

        else:
            self.turtlebot3_state_num = GET_TB3_DIRECTION


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3Drive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```
