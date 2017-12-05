# Forcecontrol

Use EGM+mini40 for forcecontrol

Yifan Hou
yifanh@cmu.edu



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


## Contact Mode
stiffness = 0.1
comp1.k=0.005, comp1.zero = 0.98, comp1.pole = 0.98
comp2.k=1, comp2.zero = 0, comp2.pole = 1

## Reference
"Experience in Force Control of Robotic Manipulators", James A. Maples and Joseph J. Becker
