# Nimesha Jayatunge Thesis

## Overview
Examining the suitability of MuJoCo physics simulation for RL training. This project makes use of the ROS2 MoveIt Task Constructor for controlling a UR5e arm + Robotiq 2F-85 gripper in MuJoCo. 

## Setup Instructions

### System Requirements
- Ubuntu 22.04
- ROS2 Humble

### Installing ROS2 Humble
Follow the official instructions to install ROS2 Humble from [here](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html#system-requirements). MoveIt2 Humble is included in this repo.

### Cloning the Repository
Clone this repository to your local machine:
```bash
git clone https://github.com/nimeshaTJ/Nimesha_Jayatunge_Thesis.git
cd Nimesha_Jayatunge_Thesis
```

### Python Requirements
- rclpy==3.3.14 
- mujoco==3.2.3
- numpy==1.21.5
- moveit-configs-utils==2.5.5
- moveit-msgs==2.2.1
- scipy==1.8.0

To install all the requirements:
```bash
pip install -r src/requirements.txt
```

### Building the Workspace
1. Source ROS install:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --executor sequential
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Packages
The following packages are used in this project:

1. **mtc_tutorial**: This package includes launch files for path planning for the UR5e arm + Robotiq 2F-85 gripper. It calso contains a python ROS node that launches a MuJoCo scene and integrates the MoveIt trajectory to move the robot arm.

2. **UR5e_robotiq_gripper_RViz**: This package configures the robot and gripper in RViz, allowing you to verify that the configuration matches the real setup.

3. **robot_moveit_config**: This package uses the previous package to set up the robot configuration in MoveIt using the MoveIt Setup Assistant, which generates this package.


## Running the Project
See [mtc_tutorial](src/mtc_tutorial/README.md) for details on how to run

## Acknowledgements
This project makes use of the following packages from [Sohaib-Snouber](https://github.com/Sohaib-Snouber):
- [mtc_tutorial](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/mtc_tutorial)
- [UR5e_robotiq_gripper_RViz](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/UR5e_robotiq_gripper_RViz)
- [robot_moveit_config](https://github.com/Sohaib-Snouber/master_project_ws/tree/main/src/robot_moveit_config)
