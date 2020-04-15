# force_control

Implementation of 6D Cartesian space hybrid force-velocity control using positional inner loop and wrist mounted FT sensor.

Author: Yifan Hou
yifanh at cmu dot edu

# Install
## Dependency
Please install the following packages:
* [cpplibrary](https://github.com/yifan-hou/cpplibrary)
* [hardware_interfaces](https://github.com/yifan-hou/hardware_interfaces) Note: this is NOT ROSControl!

## Build
This is a ROS package. Clone this repo into your catkin workspace source folder, then compile.

# How to use
Refer to `test/main.cpp` for an example.

In your own application, you should make a copy of your config file, and load your config file in your launch file.
Examples of config file are in `config/` folder, examples of launch file are in `test/` folder.

## Set force
The force being set is in transformed space (if R_a=I, then in the Tool frame).
It's the force felt by the outside from the robot. For example, if R_a=I, fz=10, the
robot will move in the +Z direction in tool frame.

The logic for force control is as follows. (see update()).
If the set force is larger than feedback(here the feedback is the force felt
by the environment, not necessarily the original sensor reading), then f_TErr is
positive, the pseudo spring-mass-damper will be pushed forward, which will
create a larger force felt by the environment to make f_TErr smaller.

Call controller.reset() before the first controller.update()

## Get Position control
If you want to disable force control in some direction, do the following in config file:
1. Set the corresponding force_fb_selection to 0 (disable feedback)
2. Set the corresponding stiffness to 0 (disable spring behavior)

## Change axis
Before you call updateAxis(), make sure you already set the pose_set and force_set.

## Reference
The implementation was initially based on
"Experience in Force Control of Robotic Manipulators", James A. Maples and Joseph J. Becker

Then a lot more functionalities are added. Please contact yifan for questions.