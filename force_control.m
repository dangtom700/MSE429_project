%{
The focus of this file is making use of created Jacobian matrix along with
other information about each component used in the setup, including the
center of mass, inertia matrix and friction parameters and an existing
trajectory profile.

The goal is to compute the force (specifically torque) of each revolute
joints, which is then used to compute the force vector on the end effector.

Secondly, the force is pick up and hold an object is 1 Newton and place
down force is 0.5 N only on the Z-axis. Find out the power to adapt to the
required force and plotting the power use to rotate each components.
%}