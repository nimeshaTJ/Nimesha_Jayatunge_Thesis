# MTC Tutorial

## Overview
The `mtc_tutorial` package demonstrates the use of the MoveIt Task Constructor (MTC) with ROS 2. This package includes launch files for path planning for the UR5e arm + Robotiq 2F-85 gripper. It calso contains a python ROS node that launches a MuJoCo scene and integrates the MoveIt trajectory to move the robot arm.

## Usage
The following should all be launched in separate terminals, with care to source the workspace in each one.

### Running MuJoCo Viewer
To play the saved trajectory, open [mujoco_node.py](src/mujoco_node.py) and ensure `record_mode = 0`. To mirror the trajectory from MoveIt, set `record_mode = 1`. Then launch the node:
```bash
python3 src/mujoco_node.py
```
### Launch RViz to display UR5e robot arm with gripper:
```bash
ros2 launch mtc_tutorial mtc_demo.launch.py
```
### Launch the controller for the robotic arm to perform pick-and-place task:
```bash
ros2 launch mtc_tutorial mtc_tutorial.launch.py
```
[mtc_tutorial.cpp](src/mtc_tutorial.cpp) can be modified to change the target poses
### Generating new stack of boxes
To generate a new stack, either of the two stack generation python scripts can be run. If differnet parameters are needed, enter the files and modify the relevant variables.
- For a 3-D random distribution, run [stack_generator_random.py](src/assets/stack_generator_random.py):
	```bash
	python3 src/assets/stack_generator_random.py
	```
- For a 2-D triangular stack, run [stack_generator_manual.py](src/assets/stack_generator_manual.py):
	```bash
	python3 src/assets/stack_generator_manual.py
	```