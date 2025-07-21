# Overview of the current state

A full pipeline from end to end of creating DH parameters to deriving forward kinematics, inverse kinematics, Jacobian joint space matrix to a smooth animation using MATLAB

## Phase

Phase 1 is about design specification and forward kinematics which shown in
> workspace_robot.m (main) defines the DH parameters, which then hard-coded a full transformation matrix from base to end effector, and making comparison to the iterative methods. At the end, generate a workspace with multiple sampling points within a define constraints of angle range
- trajectory_robot.m (supplement) creates a pseudo robotic arm to simulate the shape and movement of a robot arm based on DH parameters of choices

Phase 2 focuses on inverse kinematics and reconstructing the robot posture based on a given location and its own design constraints, found in 
> inverse_kinematics.m (main) contains a set of equations to perform inverse kinematics based on spatial location, which then cross check with the location created by forward kinematics
- simulation_STL.m (supplement) provides a visual presentation of forward and inverse kinematics of this robot configuration, which requires tuning the transformation matrix to best fit the visual representation
- import_components.m (supplement) defines the first step to introduce components for animation in a robot simulation
- rotate_component.m (supplement) is dedicated for test the movement of components during simulation
- assemble_robot.m (supplement) define the shapes of an assembled robot arm and shows the inital configuration of the robot arm
- reverse_DH.m (supplement) finds a best fit parameters for a given transformation with multiple unknowns after fine-tuning the DH parameters for simulation

Phase 3 interests in path generation, trajectory generation and the use of Jabcobian matrix in static force analysis, found in

> force_linear_drive.m (main) provides a detailed analysis of robot arm movement in space and force applied on the end effector and experimental detail such the discrepancy between analytical and geometric Jacobian matrix on linear components. The outputs are joint motion profile, end effector motion profile, joint torques during motion and animation with detailed note on the orientation of the end effector
- force_angle_drive.m (main) provides as an alternative solutions to the Jacobian approach on performing inverse kinematics using analytical way, instead of slow iteration process. The outputs are the same as the main file of this phase
- analytical_IK.m (supplement) has the first draft of how the robot arm behave in 3D space when it is driven by joint angles
- jacobian_path.m (supplement) has the first draft of how the robot arm behave in 3D space when it is driven by spatial locations
- dynamics_data.m (supplement) contains the mass properties data from Solidwork files, including mass, volume, surface area, center of mass, principle of moments, intertia tensor of center of mass, and inertia of joint
- angle_control_smlt (supplement) has a case study where there are multiple test tubes are picked and placed one by one. The goal is to witness the behavior of the robot arm in the simulation
- linear_control_smlt (supplement) has the same purpose as angle_control_smlt.m

## General procedure

### Angle driven solution
Input: a target location on 3D map

- Step 1: perform inverse kinematics and rank solutions based on 2 criteria (minimum position error and minimum displacement in Cartesian space)
- Step 2: Check the payload (state of the robot arm picking up the test tube)
- Step 3: compute in batch sets of target positions based on a constant linear velocity
- Step 4: perform the animation. In each frame of the animation,

  - Step 4.1: perform homogeneous transformation on each component to update position
  - Step 4.2: update visualization of each component in space
  - Step 4.3 (optional): create trace of end effector of sample points
  - Step 4.4: compute the Jacobian matrix numerically (using finite differences) and analytically (using derivative method) and compare the similarity
  - Step 4.5: compute the linear and angular velocity of the end effector from the trajectory
  - Step 4.6: compute the force applied on the end effector during motion
  - Step 4.7: infer the joint torque required based on payload mass and center of mass of each component
- Step 5: plot and visualize all data collected during the simulation

### Position driven solution
Input: a target location on 3D map

For every animation frame:
- Step 1: compute the direction between the current position to the target position to infer the linear velocity
- Step 2: compute the analytical Jacobian matrix for the linear velocities of the end effector
- Step 3: update the visualization of each component in space
- Step 4 (optional): create traces of end effector of sample points
- Step 5: compute the linear and angular velocity of the end effector from the trajectory
- Step 6: compute the force applied on the end effector during motion
- Step 7: infer the joint torque required based on payload mass and center of mass of each component

After the animation ends, plot and visualize all data collected during the simulation