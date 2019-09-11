# Force control

Implementation of 6D tool frame hybrid force-velocity control using positional inner loop and wrist mounted FT sensor.

Yifan Hou
yifanh@cmu.edu

# How to use
Refer to main.cpp for an example.

# Set force
The force being set is in transformed space (if R_a=I, then in the Tool frame).
It's the force felt by the outside from the robot. For example, if R_a=I, fz=10, the
robot will move in the +Z direction in tool frame.

The logic for force control is as follows. (see update()).
If the set force is larger than feedback(here the feedback is the force felt
by the environment, not necessarily the original sensor reading), then f_TErr is
positive, the pseudo spring-mass-damper will be pushed forward, which will
create a larger force felt by the environment to make f_TErr smaller.

Call controller.reset() before the first controller.update()

# Hybrid Position control
If you want to disable force control in one of x, y or z direction,
1. Set the corresponding force_fb_selection to 0 (disable feedback)
2. Set the corresponding stiffness to 0 (disable spring behavior)

# Change axis
Before you call updateAxis(), make sure you already set the pose_set and force_set.

# Reference
The implementation is initially based on
"Experience in Force Control of Robotic Manipulators", James A. Maples and Joseph J. Becker

Then a lot more functionalities are added.