# Overview of the current state

## Structure
All MATLAB files (.m files) are standalone files. They do not rely on each other to function as a working program. There are 3 types of files to look for in this folder

- STL file (3D objects): These contain the links to assemble together in MATLAB (refer to import_component.m, assemble_robot.m, and simulation_STL.m)
- MATLAB data: These are saved to hard disk to speed up the simulation work in MATLAB
- MATLAB code (.m files): Theses are standalone file that serve a unique purpose as the name suggested

## Overview of each .m files

- assemble_robot: This file assembles together the robot to its initial state in the simulation. It contains the reference points of each STL components and map to desired position (target location) in vector space.
- import_component: This file loads components are a raw data, then do necessary transformation to simply the work of assembly in the assemble_robot.m file and simulation_STL.m file.
- inverse kinematics: This file contains a set of equations that solve for a set of angle solutions given only the XYZ coordinate in the universal frame as a function. In the main loop, there is an experiment that tries to forward kinematics to yield the XYZ coordinate, which is then applied kinematics to test the accuracies of the best angle configurations that matched the desired position in 3D space.
- rotate_component: This file shows the work needed to rotate STL components in space. It is essentially another perspective of looking at how DH transformation works intuitively, including shifting objects to an origin, rotate around that origin and translate back to a desired location. It also serves as a ground work for implementing more complex simulation in simulation_STL.m file.
- simulation_STL: This file includes forward kinematics of a fully assembled robot arm. There are 3 angular that are free to control for simulation. The simulation also draws the coordinate system are the component moves in space. The simulation gives information about the path it travels and the behavior of the relative coordinate systems work over time.
- trajectory_robot: This file contain a simplified model to the simulation the path it creates over time as it forward kinematics, utilizing a complete base to end-effector transformation matrix from the workspace_robot.m file.
- workspace_robot: This file contains the algorithm of direct DH transformation and a full transformation matrix from base to end-effector based on the parameters provided within this file. There is also a graphical illustration of the error between in the direct DH transformation and a full matrix transformation from base to end-effector given the DH configurations of this file. The angles in this file has limited angle range, which is iterate and permutate to create a clear workspace or a clear vision of where the end-effector is likely to be located in 3D space, in additions to the 3 side projection of the view.