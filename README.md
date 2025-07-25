# Pabot - An autonomous patrol robot based on ROS 2 and Navigation 2 (Isaac Sim 4.5)

This is an autonomous patrol robot project based on ROS 2 Humble and Navigation 2, it runs in the simulation environment of **Isaac Sim 4.5**. 

If you want to view a different version for **Gazebo**, please go to: (https://github.com/baz1boy/AutoPatrol-Robot-ROS2)

First, open the Isaac Sim GUI and click Play, then use the one-click option to load the Navigation System and Application. The robot can then autonomously patrol between predefined waypoints in a mapped environment.

### **Demo Video**:[![YouTube](https://img.shields.io/badge/YouTube-Watch-red?logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=nPYWb86pzSc&list=PLG0yEiqorTkghIX3G7IwC2tOvI9JmIdd0&index=2)

<div align="center">
  <a href="https://www.youtube.com/watch?v=nPYWb86pzSc&list=PLG0yEiqorTkghIX3G7IwC2tOvI9JmIdd0&index=2">
    <img src="https://img.youtube.com/vi/nPYWb86pzSc/0.jpg" width="480">
    <br>
    🎥 Click to watch the demo video
  </a>
</div>

## 1. Project Introduction

This project implements a simulation function for an autonomous patrol robot based on ROS 2 and Navigation 2.

### 1.1 GUI Application:
We convert the compiled `xacro` file into a `urdf` file, then use Isaac Sim's URDF Importer to convert it into a `usd` file. 
In Isaac Sim, we perform the corresponding configuration of actuators and sensors, and add the **Action Graph** accordingly:
- Add the ROS 2 differential drive control action graph (Twist)
- Add the ROS 2 clock publisher
- Add the ROS 2 camera helper
- Add the ROS 2 RTX Lidar helper
- Add ROS 2 odometry and TF tree
- Add a suitable map and use Tools > Robotics > Occupancy Map to perform mapping and save the result.

**Note**: Please move the camera and lidar to their corresponding links, in this project: `camera_optical_link` and `laser_link`. This is not a straightforward process, and the specific steps require referring to the official Isaac Sim documentation.

### 1.2 ROS2 Packages:
The functions of each package are as follows:
<pre>autopatrol_robot           # Functional package for autonomous patrol implementation
autopatrol_interfaces      # Interfaces related to the autonomous patrol system
pabot_robot_navigation2    # Navigation configuration package for the robot
</pre>

## 2. Usage Instruction

The development environment for this project is as follows:

*   **Operating System**:Ubuntu 22.04
*   **Core Application**: Isaac Sim 4.5.0 
*   **ROS Version**: ROS 2 Humble
*   **Core Languages**: Python

### 2.1 Installation

This project uses **Isaac Sim** and **Navigation2** for simulation, control and navigation. Please install the required dependencies before building the project using the following commands:

1. **Install Isaac Sim**   
The most **IMPORTANT** step is to install **Isaac Sim 4.5**, please check the requirements and follow the installation Instruction: (https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)

2. **Install Navigation 2**
```bash
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup 
```

3. **Install speech synthesis and image-related packages**
```bash
sudo apt install python3-pip  -y
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
pip install gTTS
sudo apt install mpg123
# sudo apt install espeak-ng -y
# sudo pip3 install espeakng
```

### 2.2 Run

After installing the dependencies, you can build the project in the workspace directory using the colcon tool:

**Build the packages**:
```bash
colcon build
source install/setup.bash
```

**Observe the robot**:
After launch, the robot will automatically initialize and then begin navigating sequentially to the waypoints defined in the `autopatrol_robot/config/patrol_params.yaml` file. Before and after reaching each point, you will hear a voice prompt. Upon arrival at each target point, the robot will take a photo and announce the event via speech. The newly captured images can be found in the `pabot_ws/pabot_images/` directory.

#### 2.2.1 One-Click Run

**One-click Launch**:
- Make sure you have opened **Isaac Sim** and loaded the pabot stage, then click **Play** to start the simulation.

```bash
ros2 launch autopatrol_robot pabot_isaac_all.launch.py
```
This command will:
- Launch the full Nav2 navigation stack, including AMCL localization, planner, controller, etc.
- Launch `patrol_node` to begin executing the patrol task.
- Launch `speaker` to wait for speech playback requests.

#### 2.2.2 Run Step by Step

Alternatively, you can launch each component in separate terminals:

1.  **Launch Simulation**
```bash
cd ~/isaac_sim
./isaac_sim.sh
```

2.  **Launch Navigation**
```bash
source install/setup.bash
ros2 launch patrol_robot_navigation2 isaac_navigation2.launch.py
```

3.  **Launch Autopatrol Robot**
```bash
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

### 2.3 Customize Patrol Path and Speed

*   **Modify Patrol Points**:
    Edit the `autopatrol_robot/config/patrol_params.yaml` file and modify the `target_points` list to customize the patrol path.

*   **Adjust Robot Speed**:
    Edit the `patrol_robot_navigation2/config/nav2_params.yaml` file. You can adjust parameters under `controller_server` → `FollowPath` and `velocity_smoother` to modify the robot’s maximum and minimum driving speeds.

## 3. Author

- [bazi](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)
