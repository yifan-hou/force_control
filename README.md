# Forcecontrol

Use EGM+mini40 for forcecontrol

Yifan Hou
yifanh@cmu.edu

# Set force
The force being set is in transformed space (if R_a=I, then in world frame).
It's the force felt by the outside from the robot. For example, if fz=10, the
robot will pull up.

The logic for force control is as follows. (see update()).
If the set force is larger than feedback(here the feedback is the force felt
by the environment, not necessarily the original sensor reading), then f_TErr is
positive, the pseudo spring-mass-damper will be pushed forward, which will
create a larger force felt by the environment to make f_TErr smaller.

Call controller.reset() before the first controller.update()

# Compensator Parameters
## Teach Mode
* Comp1
** k: T/m
** zero: 0
** pole: 1 - alpha\*T/m
* Comp2
** k: 1000\*T
** zero: 0
** pole: 1
* Meaning of variables
** T: controller period. e.g. For 500Hz, T=0.002
** m: imaginary mass of end effector in kg. e.g. m=2
** alpha: imaginary viscous friction coefficient in Ns/m. e.g. alpha=50
* Typical choices
** 500Hz, 5kg, alpha=50
*** comp1: k=0.0008, zero=0, pole=0.98
*** comp2: k=1, zero=0, pole=1
** 500Hz, 1kg, alpha=10
*** comp1: k=0.004, zero=0, pole=0.98
*** comp2: k=1, zero=0, pole=1


## Contact Mode (deprecated)
stiffness = 0.1
comp1.k=0.005, comp1.zero = 0.98, comp1.pole = 0.98
comp2.k=1, comp2.zero = 0, comp2.pole = 1



##Hybrid Position control
If you want to disable force control in one of x, y or z direction,
1. Set the corresponding force_fb_selection to 0 (disable feedback)
2. Set the corresponding stiffness to 0 (disable spring behavior)

##Change axis
Before you call updateAxis(), make sure you already set the pose_set and force_set.

## Reference
"Experience in Force Control of Robotic Manipulators", James A. Maples and Joseph J. Becker
