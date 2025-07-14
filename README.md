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