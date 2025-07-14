close all; clc; clear all;

%% -------------------- DYNAMIC PROPERTIES (SI UNITS) --------------------
g = 9.81; % m/s^2

% Link 1 (Base Link)
Link1.mass = 0.21025; % kg
Link1.com = [0.00512, 0.00097, 0.05928]; % m (relative to joint1 frame)
Link1.inertia_com = [371584.58,  1133.32,   37190.55;
                     1133.32,   390041.76,  8369.32;
                     37190.55,   8369.32,   129419.43] * 1e-9; % kg·m^2

% Link 2 (Middle Link)
Link2.mass = 0.11181; % kg
Link2.com = [0.00138, 0.10492, 0.00001]; % m (relative to joint2 frame)
Link2.inertia_com = [230119.67, -26620.50,  85.99;
                    -26620.50,   53290.16,  76.49;
                     85.99,      76.49,     250996.49] * 1e-9; % kg·m^2

% Link 3 (End-Effector Link)
Link3.mass = 0.03816; % kg
Link3.com = [0.01352, 0.06237, 0.00001]; % m (relative to joint3 frame)
Link3.inertia_com = [67677.77, -487.23,  1.61;
                    -487.23,   3902,     9.04;
                     1.61,     9.04,     65335.24] * 1e-9; % kg·m^2

% Test tube properties
test_tube.mass = 0.00668; % kg
test_tube.com = [0, 0, -0.04123]; % m (relative to end-effector)

% Store all links
robot.links = {Link1, Link2, Link3};
robot.num_links = 3;

%% -------------------- SETUP FIGURE --------------------
figure('Renderer', 'zbuffer');
axis equal; axis vis3d; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); rotate3d on;
axis([-300 300 -300 300 -50 200]);
title("Force-Controlled Path Planning with Torque Analysis")

% Lighting setup
light('Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;

%% -------------------- DEFINE COLORS --------------------
link_colors  = {[0.6 0.6 0.4], [0.4 0.6 0.8], [0.8 0.4 0.6]};
frame_colors = {'r', 'g', 'b'};

%% -------------------- JOINT & LINK SETUP --------------------
joint1 = [0, 0, 0];
joint2 = [1.3, 40, 95];
joint3 = [-132, 12.5, 95.5];
end_effector = [-258.994, 0.26451, 98.3614];

d12 = joint2 - joint1;
d23 = joint3 - joint2;
d3ee = end_effector - joint3;

[L1.F, L1.V] = load_link('Link1.STL', [97.9527, 34.4903, 134.099], eye(3));
[L2.F, L2.V] = load_link('Link2.STL', [157.325, 42.4651, 173.659], [1 0 0; 0 0 -1; 0 1 0]);
[L3.F, L3.V] = load_link('Link3.STL', [127.04, 113.902, 187.562], [1 0 0; 0 0 -1; 0 1 0]);

L1.V0 = L1.V';
L2.V0 = L2.V';
L3.V0 = L3.V';

q(1) = patch('Faces', L1.F, 'Vertices', L1.V0', 'FaceColor', link_colors{1}, 'EdgeColor', 'none');
q(2) = patch('Faces', L2.F, 'Vertices', L2.V0', 'FaceColor', link_colors{2}, 'EdgeColor', 'none');
q(3) = patch('Faces', L3.F, 'Vertices', L3.V0', 'FaceColor', link_colors{3}, 'EdgeColor', 'none');

%% -------------------- ANIMATION --------------------
trace_pts = struct('L1', [], 'L2', [], 'L3', [], 'Lee', []);
samples = create_samples([0,50,100], [150,150,100], [-150,100,100], 100, 5);
[num_samples, ~] = size(samples);
disp("Generated sample points")
disp(samples)
disp("--------------------------------------------------------------");
r = 5;

% Motion parameters
steps = 20;
dt = 0.1;            % Time step [s]
safety_margin = 5;  % degrees from constraint boundary
angle_constaints = [-300 -120 -250; 300 120 250];
constraints = deg2rad(angle_constaints);
position_tolerance = 1;  % 1 mm position tolerance
current_angle = [270,-66,156];  % degrees
current_time = 0;         % Current simulation time

% Force application parameters
pickup_index = 3;          % When to pick up object
place_index = num_samples - 2; % When to place object
pickup_force = [0, 0, 0.2];  % N (upward during pickup)
place_force = [0, 0, -0.2];% N (downward during placement)
hold_force = -test_tube.com * test_tube.mass; % N (upward during hold)

% Initialize data recording arrays
all_times = zeros(steps*num_samples, 3);          % Put time stamp
all_angles = zeros(steps*num_samples, 3);         % Put current angles
all_angular_vel = zeros(steps*num_samples, 3);    % Put angle_vel of EE
all_J_error = zeros(steps*num_samples, 3);        % Put similarity score J
all_location = zeros(steps*num_samples, 3);       % Put x, y, z of EE
all_linear_vel = zeros(steps*num_samples, 3);     % Put linear_vel of EE
all_torques = zeros(steps*num_samples, 3);
all_gravity_torques = zeros(steps*num_samples, 3);
all_ext_torques = zeros(steps*num_samples, 3);
current_load = 0;          % Track if carrying object
applied_force = [0, 0, 0]; % Current external force

% Animation and dynamics loop
for i = 1:num_samples
    sample = samples(i,:);
    px_target = sample(1);
    py_target = sample(2);
    pz_target = sample(3);

    [bx, by, bz] = sphere(20);
    sx = r * bx + px_target;
    sy = r * by + py_target;
    sz = r * bz + pz_target;

    fprintf("New location target: %.4f,%.4f,%.4f\n", px_target, py_target, pz_target);

    all_solutions = inverseKinematics(px_target, py_target, pz_target);
    disp("Inverse kinematics solutions:")
    disp(rad2deg(all_solutions))
    best_solution = [];
    min_coordinate_error = [];
    min_error = Inf;
    min_joint_distance = Inf;
    
    % ========== SOLUTION SELECTION LOGIC ========== %
    % First pass: find minimum position error
    for j = 1:size(all_solutions, 1)
        sol_deg = rad2deg(all_solutions(j,:));
        sol_deg = filter_angle(angle_constaints, sol_deg);
        if isempty(sol_deg)
            continue;
        end
        
        T = BaseToTool(sol_deg(1), sol_deg(2), sol_deg(3));
        coordinate_error = [px_target, py_target, pz_target] - T(1:3,4)';
        position_error = norm(coordinate_error);
        
        if position_error < min_error
            min_error = position_error;
        end
    end
    
    % Second pass: find solution with smallest joint space distance
    for j = 1:size(all_solutions, 1)
        sol_deg = rad2deg(all_solutions(j,:));
        sol_deg = filter_angle(angle_constaints, sol_deg);
        if isempty(sol_deg)
            continue;
        end
        
        T = BaseToTool(sol_deg(1), sol_deg(2), sol_deg(3));
        coordinate_error = [px_target, py_target, pz_target] - T(1:3,4)';
        position_error = norm(coordinate_error);
        
        if position_error <= min_error + position_tolerance
            joint_diff = wrapTo180(sol_deg - current_angle);
            joint_distance = norm(joint_diff);
            
            if joint_distance < min_joint_distance
                min_joint_distance = joint_distance;
                min_coordinate_error = coordinate_error;
                best_solution = sol_deg;
            end
        end
    end

    if isempty(best_solution)
        disp("No valid solution. Skipping point.")
        disp("--------------------------------------------------------------");
        continue
    end
    fprintf("Distance error: %.4f\n", min_error);
    fprintf("Target angle (deg): %.4f,%.4f,%.4f\n", best_solution(1), best_solution(2), best_solution(3));
    fprintf("Coordinate error (mm): %.4f,%.4f,%.4f\n", min_coordinate_error(1), min_coordinate_error(2), min_coordinate_error(3));

    target_ball = surf(sx, sy, sz, 'FaceColor', 'magenta', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    drawnow;
    
    % ======== UPDATE PAYLOAD STATE ======== %
    % Apply pickup/placement forces
    if i == pickup_index
        current_load = test_tube.mass;
        applied_force = pickup_force;
        fprintf("---- PICKUP EVENT: Applying +Z-force ----\n");
    elseif i > pickup_index && i < place_index
        current_load = test_tube.mass;
        applied_force = hold_force;
        fprintf("---- HOLDING EVENT: Applying +Z-force ----\n");
    elseif i == place_index
        current_load = 0;
        applied_force = place_force;
        fprintf("---- PLACEMENT EVENT: Applying -Z-force ----\n");
    else
        applied_force = [0, 0, 0];
    end
    
    % ========== VELOCITY CONTROL ========== %
    delta_angle = wrapTo180(best_solution - current_angle);
    theta = zeros(3, steps);
    
    for s = 1:steps
        progress = s / steps;
        target_angles = current_angle + delta_angle * progress;
        
        for joint = 1:3
            dist_lower = target_angles(joint) - angle_constaints(1, joint);
            dist_upper = angle_constaints(2, joint) - target_angles(joint);
            
            if dist_lower < safety_margin
                reduction = (dist_lower / safety_margin)^2;
                target_angles(joint) = current_angle(joint) + ...
                    delta_angle(joint) * progress * reduction;
            elseif dist_upper < safety_margin
                reduction = (dist_upper / safety_margin)^2;
                target_angles(joint) = current_angle(joint) + ...
                    delta_angle(joint) * progress * reduction;
            end
        end
        
        theta(:, s) = deg2rad(target_angles);
    end
    
    current_angle = best_solution;

    for k = 1:steps
        % Record current joint angles
        theta_rad = theta(:, k)';
        current_angles_deg = rad2deg(theta_rad);
        all_angles(k + (i-1)*steps, :) = current_angles_deg;
        all_times(k + (i-1)*steps, :) = current_time;
        previous_angle = current_angles_deg; % Update for next comparison
        current_time = current_time + dt;
        
        % Update visualization
        T1 = [rotz(theta(1,k)), [0;0;0]; 0 0 0 1];
        T2 = [roty(theta(2,k)), d12'; 0 0 0 1];
        T3 = [roty(theta(3,k)), d23'; 0 0 0 1];
        Tee = [[1,0,0;0,1,0;0,0,1], d3ee'; 0 0 0 1];

        V1 = apply_transform(L1.V0, T1);
        V2 = apply_transform(L2.V0, T1*T2);
        V3 = apply_transform(L3.V0, T1*T2*T3);

        set(q(1), 'Vertices', V1');
        set(q(2), 'Vertices', V2');
        set(q(3), 'Vertices', V3');
        
        % Extract frame data
        joint1 = T1;
        z1 = T1(1:3, 3);
        joint1_pos = T1(1:3,4)';
        joint2 = joint1 * T2;
        z2 = joint2(1:3, 2);
        joint2_pos = joint2(1:3,4)';
        joint3 = joint2 * T3;
        z3 = joint3(1:3, 2);
        joint3_pos = joint3(1:3,4)';
        jointee = joint3 * Tee;
        jointee_pos = jointee(1:3,4)';
        all_location(k + (i-1)*steps, :) = jointee_pos;

        trace_pts.L1 = [trace_pts.L1; joint1_pos];
        trace_pts.L2 = [trace_pts.L2; joint2_pos];
        trace_pts.L3 = [trace_pts.L3; joint3_pos];
        trace_pts.Lee = [trace_pts.Lee; jointee_pos];

        % Draw traces
        plot3(trace_pts.L1(:,1), trace_pts.L1(:,2), trace_pts.L1(:,3), 'r.', 'MarkerSize', 1);
        plot3(trace_pts.L2(:,1), trace_pts.L2(:,2), trace_pts.L2(:,3), 'g.', 'MarkerSize', 1);
        plot3(trace_pts.L3(:,1), trace_pts.L3(:,2), trace_pts.L3(:,3), 'b.', 'MarkerSize', 1);
        plot3(trace_pts.Lee(:,1), trace_pts.Lee(:,2), trace_pts.Lee(:,3), 'b.', 'MarkerSize', 1);
        
        % Draw coordinate frames
        draw_frame(T1, 30);
        draw_frame(joint2, 30);
        draw_frame(joint3, 30);
        draw_frame(jointee, 30);
        
        drawnow;

        % Calculate force applied
        Jk = Jacobian(current_angles_deg(1), current_angles_deg(2), current_angles_deg(3));
        cross1 = cross(z1, (jointee_pos - joint1_pos)');
        cross2 = cross(z2, (jointee_pos - joint2_pos)');
        cross3 = cross(z3, (jointee_pos - joint3_pos)');
        J_cross = [cross1, cross2, cross3];
        diff_J = J_cross - Jk;
        norm_diff = norm(diff_J, 'fro');
        norm_Jk = norm(Jk, 'fro');
        norm_J_cross = norm(J_cross, 'fro');
        denom = (norm_Jk + norm_J_cross) / 2;
        rel_norm_error = norm_diff / (denom + eps);
        all_J_error(k + (i-1)*steps, :) = rel_norm_error;

        J_full = [J_cross; [z1, z2, z3]];
        wrench_vec = J_full * theta_rad';
        all_linear_vel(k + (i-1)*steps, :) = wrench_vec(1:3);
        all_angular_vel(k + (i-1)*steps, :) = wrench_vec(4:6);

        % ==================== DYNAMICS ANALYSIS ====================
        % Convert Jacobian to meters (from mm to m)
        J_ee_m = Jk / 1000;
        
        % Compute torque due to external force
        ext_torque = J_ee_m' * applied_force';
        
        % Compute gravity compensation torque using exact CoM Jacobians
        gravity_torque = zeros(3,1);
        
        % Calculate CoM positions in base frame
        % Link 1 CoM (in base frame)
        com1_base = T1(1:3,1:3) * Link1.com' * 1000;  % Convert m to mm
        J_com1 = compute_com_jacobian_geometric(...
            [joint1_pos; joint2_pos; joint3_pos],...
            [z1, z2, z3],...
            com1_base...
        ) / 1000;  % Convert mm to m
        
        % Link 2 CoM (in base frame)
        com2_base = joint2(1:3,1:3) * Link2.com' * 1000 + joint2(1:3,4);
        J_com2 = compute_com_jacobian_geometric(...
            [joint1_pos; joint2_pos; joint3_pos],...
            [z1, z2, z3],...
            com2_base...
        ) / 1000;  % Convert mm to m
        
        % Link 3 CoM (in base frame)
        com3_base = joint3(1:3,1:3) * Link3.com' * 1000 + joint3(1:3,4);
        J_com3 = compute_com_jacobian_geometric(...
            [joint1_pos; joint2_pos; joint3_pos],...
            [z1, z2, z3],...
            com3_base...
        ) / 1000;  % Convert mm to m
        
        % Calculate gravity torque for each link
        gravity_force1 = [0; 0; -Link1.mass * g];
        gravity_force2 = [0; 0; -Link2.mass * g];
        gravity_force3 = [0; 0; -Link3.mass * g];
        
        gravity_torque = gravity_torque + ...
            J_com1' * gravity_force1 + ...
            J_com2' * gravity_force2 + ...
            J_com3' * gravity_force3;
        
        % Add torque due to test tube if carried (at end-effector)
        if current_load > 0
            tube_gravity_force = [0; 0; -current_load * g];
            gravity_torque = gravity_torque + J_ee_m' * tube_gravity_force;
        end
        
        % Total torque = gravity compensation + external forces
        total_torque = gravity_torque + ext_torque;
        
        % Store torque values
        all_torques(k + (i-1)*steps, :) = total_torque';
        all_gravity_torques(k + (i-1)*steps, :) = gravity_torque';
        all_ext_torques(k + (i-1)*steps, :) = ext_torque';
    end

    disp("--------------------------------------------------------------");
end

%% -------------------- PLOT JOINT DATA --------------------
% Trim unused elements in recording arrays
valid_indices = all_times(:,1) ~= 0;
all_times = all_times(valid_indices, :);
all_angles = all_angles(valid_indices, :);
all_angular_vel = all_angular_vel(valid_indices, :);
all_J_error = all_J_error(valid_indices, :);
all_location = all_location(valid_indices, :);
all_linear_vel = all_linear_vel(valid_indices, :);
all_torques = all_torques(valid_indices, :);
all_gravity_torques = all_gravity_torques(valid_indices, :);
all_ext_torques = all_ext_torques(valid_indices, :);

% Calculate angular velocity using central difference
n = size(all_angles, 1);
velocity = zeros(n, 3);  % Preallocate velocity array

% Compute angular velocity (deg/s)
for j = 1:3  % For each joint
    for i = 2:n-1  % Avoid boundaries
        velocity(i, j) = (all_angles(i+1, j) - all_angles(i-1, j)) / (2*dt);
    end
    
    % Handle boundaries with forward/backward differences
    if n > 1
        % First point: forward difference
        velocity(1, j) = (all_angles(2, j) - all_angles(1, j)) / dt;
        
        % Last point: backward difference
        velocity(n, j) = (all_angles(n, j) - all_angles(n-1, j)) / dt;
    end
end

% Calculate angular acceleration using central differences
acceleration = zeros(n, 3);  % Preallocate acceleration array

% Compute angular acceleration (deg/s^2)
for j = 1:3  % For each joint
    for i = 2:n-1  % Avoid boundaries
        acceleration(i, j) = (velocity(i+1, j) - velocity(i-1, j)) / (2*dt);
    end
    
    % Handle boundaries with forward/backward differences
    if n > 1
        % First point: forward difference
        acceleration(1, j) = (velocity(2, j) - velocity(1, j)) / dt;
        
        % Last point: backward difference
        acceleration(n, j) = (velocity(n, j) - velocity(n-1, j)) / dt;
    end
end

figure;
plot(all_times, all_J_error, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Relative Frobenius Error');
title('Jacobian Discrepancy');
grid on;

figure;
axis_name = {'\alpha', '\beta', '\gamma'};
for i = 1:3
    subplot(3, 3, i);
    plot(all_times, all_angular_vel(:, i), 'LineWidth', 2);
    title([axis_name{i} ' - Angular Velocity']);
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    grid on;

    subplot(3, 3, i+3);
    plot(all_times, all_linear_vel(:, i), 'LineWidth', 2);
    title([axis_name{i} ' - Linear Velocity']);
    xlabel('Time (s)');
    ylabel('Linear Velocity (mm/s)');
    grid on;

    subplot(3, 3, i+6);
    plot(all_times, all_location(:, i), 'LineWidth', 2);
    title([axis_name{i} ' - Spatial Position']);
    xlabel('Time (s)');
    ylabel('Location (mm)');
    grid on;
end
sgtitle('End Effector Motion Profile');

% Create plots
joint_names = {'Joint 1', 'Joint 2', 'Joint 3'};

figure;
for j = 1:3
    % Angular Displacement
    subplot(3,3,j);
    plot(all_times, all_angles(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - Angular Displacement']);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    grid on;
    
    % Angular Velocity
    subplot(3,3,j+3);
    plot(all_times, velocity(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - Angular Velocity']);
    xlabel('Time (s)');
    ylabel('Velocity (deg/s)');
    grid on;
    
    % Angular Acceleration
    subplot(3,3,j+6);
    plot(all_times, acceleration(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - Angular Acceleration']);
    xlabel('Time (s)');
    ylabel('Acceleration (deg/s^2)');
    grid on;
end

sgtitle('Joint Motion Profiles');

%% -------------------- PLOT TORQUE ANALYSIS --------------------
figure;
for j = 1:3
    % Angular Displacement
    subplot(3,3,j);
    plot(all_times, all_gravity_torques(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - Gravity Compensation']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    grid on;
    xline(all_times((pickup_index-1)*steps), 'g--', 'Pickup', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xline(all_times((place_index-1)*steps), 'm--', 'Placement', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Angular Velocity
    subplot(3,3,j+3);
    plot(all_times, all_ext_torques(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - External Forces']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    grid on;
    xline(all_times((pickup_index-1)*steps), 'g--', 'Pickup', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xline(all_times((place_index-1)*steps), 'm--', 'Placement', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Angular Acceleration
    subplot(3,3,j+6);
    plot(all_times, all_torques(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - Total Torques']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    grid on;
    xline(all_times((pickup_index-1)*steps), 'g--', 'Pickup', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xline(all_times((place_index-1)*steps), 'm--', 'Placement', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

sgtitle('Joint Torques During Motion');

%% -------------------- FUNCTIONS --------------------
function [F, V] = load_link(filename, ref_point, R_align)
    data = stlread(filename);
    V = (R_align * (data.Points - ref_point)')';
    F = data.ConnectivityList;
end

function V_out = apply_transform(V_in, T)
    V_hom = [V_in; ones(1, size(V_in,2))];
    V_out_hom = T * V_hom;
    V_out = V_out_hom(1:3,:);
end

function R = rotz(theta_rad)
    c = cos(theta_rad); s = sin(theta_rad);
    R = [c -s 0; s c 0; 0 0 1];
end

function R = roty(theta_rad)
    c = cos(theta_rad); s = sin(theta_rad);
    R = [c 0 s; 0 1 0; -s 0 c];
end

function draw_frame(T, L)
    origin = T(1:3,4);
    axes = T(1:3,1:3) * diag([L L L]);
    colors = {'r','g','b'};
    for i = 1:3
        line([origin(1) origin(1)+axes(1,i)], ...
             [origin(2) origin(2)+axes(2,i)], ...
             [origin(3) origin(3)+axes(3,i)], ...
             'Color', colors{i}, 'LineWidth', 2);
    end
end

function angle = wrapTo180(angle)
    angle = mod(angle + 180, 360) - 180;
end

function filtered_solution = filter_angle(constraints, solution)
    filtered_solution = []; % Default output

    if isempty(solution)
        return;
    end

    for i = 1:3
        if (solution(i) < constraints(1,i) || solution(i) > constraints(2,i))
            return
        end
    end

    filtered_solution = solution;
end

function samples_path = create_samples(knot_point, start_of_arc_point, end_of_arc_point, drop_point_z_offset, N_arc_samples)
    % Define radius boundaries
    r_min = 50;
    r_max = 347;
    
    % Initialize samples_path with the knot point
    samples_path = knot_point;
    
    % Validate and add start-of-arc point
    if ~is_valid_point(start_of_arc_point, r_min, r_max)
        error('Start-of-arc point is out of valid radius boundary [50, 300].');
    end
    samples_path = [samples_path; start_of_arc_point];
    
    % Create and validate drop point for start-of-arc
    start_drop = [start_of_arc_point(1), start_of_arc_point(2), start_of_arc_point(3) - drop_point_z_offset];
    if ~is_valid_point(start_drop, r_min, r_max)
        error('Start-of-arc drop point is out of valid radius boundary [50, 300].');
    end
    samples_path = [samples_path; start_drop];
    
    % Convert start and end points to spherical coordinates
    [az_start, el_start, r_start] = cart2sph(start_of_arc_point(1), start_of_arc_point(2), start_of_arc_point(3));
    [az_end, el_end, r_end] = cart2sph(end_of_arc_point(1), end_of_arc_point(2), end_of_arc_point(3));
    
    % Adjust azimuth for shortest path
    az_diff = az_end - az_start;
    if abs(az_diff) > pi
        if az_diff > 0
            az_end_adjusted = az_end - 2*pi;
        else
            az_end_adjusted = az_end + 2*pi;
        end
    else
        az_end_adjusted = az_end;
    end
    
    % Generate interpolated arc samples
    r_arc = linspace(r_start, r_end, N_arc_samples);
    az_arc = linspace(az_start, az_end_adjusted, N_arc_samples);
    el_arc = linspace(el_start, el_end, N_arc_samples);
    
    arc_points = zeros(N_arc_samples, 3);
    for i = 1:N_arc_samples
        % Wrap angles to [-pi, pi] and convert to Cartesian
        az_i = wrapToPi(az_arc(i));
        el_i = wrapToPi(el_arc(i));
        [x, y, z] = sph2cart(az_i, el_i, r_arc(i));
        arc_points(i,:) = [x, y, z];
        
        % Validate generated point
        if ~is_valid_point(arc_points(i,:), r_min, r_max)
            error('Generated arc point is out of valid radius boundary [50, 300].');
        end
    end
    samples_path = [samples_path; arc_points];
    
    % Create and validate drop point for end-of-arc
    end_drop = [end_of_arc_point(1), end_of_arc_point(2), end_of_arc_point(3) - drop_point_z_offset];
    if ~is_valid_point(end_drop, r_min, r_max)
        error('End-of-arc drop point is out of valid radius boundary [50, 300].');
    end
    samples_path = [samples_path; end_drop];
    
    % Add end-of-arc point and return to knot point
    samples_path = [samples_path; end_of_arc_point; knot_point];
end

% Helper function to check point validity
function is_valid = is_valid_point(point, r_min, r_max)
    r = norm(point);
    is_valid = (r >= r_min && r <= r_max);
end

% Helper function to wrap angles to [-pi, pi]
function angle_wrapped = wrapToPi(angle)
    angle_wrapped = mod(angle + pi, 2*pi) - pi;
end

function T = BaseToTool(theta1_deg, theta2_deg, theta3_deg)
    % Convert degrees to radians
    t1 = deg2rad(theta1_deg);
    t2 = deg2rad(theta2_deg);
    t3 = deg2rad(theta3_deg);
    
    % Precompute sines and cosines
    C1 = cos(t1); S1 = sin(t1);
    C2 = cos(t2); S2 = sin(t2);
    
    C23 = cos(t2 + t3);
    S23 = sin(t2 + t3);
    
    % Position equations (from ^0_{ee}T)
    x = C1*(2.8614*S23 - 126.994*C23 - 133.3*C2 + 0.5*S2 + 1.3) - 0.2645*S1;
    y = S1*(2.8614*S23 - 126.994*C23 - 133.3*C2 + 0.5*S2 + 1.3) + 0.2645*C1;
    z = 126.994*S23 + 2.8614*C23 + 0.5*C2 + 133.3*S2 + 95;
    
    % Rotation matrix (simplified from ^0_{ee}T)
    R = [C1*C23, -S1, C1*S23;
         S1*C23,  C1, S1*S23;
         -S23,     0, C23];

    % Assemble homogeneous transformation matrix
    T = [R, [x; y; z];
         0, 0, 0, 1];
end

function solutions = inverseKinematics(px, py, pz)
    R = 2.8614^2 + 126.994^2;
    C = 133.3^2 + 0.5^2;

    solutions = [];
    
    r_squared = px^2 + py^2;
    if r_squared < 0.2645^2
        return;  % No valid theta1
    end
    
    alpha = atan2(px, -py);
    beta = acos(-0.2645 / sqrt(r_squared));
    theta1_candidates = [alpha + beta, alpha - beta];

    for t1 = theta1_candidates
        c1 = cos(t1); s1 = sin(t1);
        U = c1*px + s1*py - 1.3;
        V = pz - 95;
        M = 133.3*U - 0.5*V;
        N = -0.5*U - 133.3*V;
        L = 0.5 * (R - C - U^2 - V^2);

        if M^2 + N^2 < L^2
            continue;  % No real solution for theta2
        end
        
        alpha2 = atan2(N, M);
        beta2 = acos(L / sqrt(M^2 + N^2));
        theta2_candidates = [alpha2 + beta2, alpha2 - beta2];

        for t2 = theta2_candidates
            c2 = cos(t2); s2 = sin(t2);
            P = U + 133.3*c2 - 0.5*s2;
            Q = V - 0.5*c2 - 133.3*s2;

            s23 = (2.8614*P + 126.994*Q) / R;
            c23 = (-126.994*P + 2.8614*Q) / R;
            t3 = atan2(s23, c23) - t2;

            % Convert to degrees for output
            solutions = [solutions;
                         rad2deg(t1), rad2deg(t2), rad2deg(t3)];
        end
    end

    solutions = deg2rad(solutions);
end

function J = Jacobian(theta1_deg, theta2_deg, theta3_deg)
    t1 = deg2rad(theta1_deg);
    t2 = deg2rad(theta2_deg);
    t3 = deg2rad(theta3_deg);
    
    C1 = cos(t1); S1 = sin(t1);
    C2 = cos(t2); S2 = sin(t2);
    C23 = cos(t2 + t3); S23 = sin(t2 + t3);
    
    % Compute components
    D = 2.8614*C23 + 126.994*S23;
    B = D + 133.3*S2 + 0.5*C2;
    F = 126.994*C23 - 2.8614*S23;
    E = F - 0.5*S2 + 133.3*C2;
    A = -E + 1.3;
    
    % Jacobian matrix
    J = [ -S1*A - 0.2645*C1,  C1*B, C1*D;
           C1*A - 0.2645*S1,  S1*B, S1*D;
           0,                E,    F ];
end

function Jv = compute_com_jacobian_geometric(joint_positions, joint_axes, com_position)
    % joint_positions: 3x3 matrix [j1_pos; j2_pos; j3_pos] (in mm)
    % joint_axes: 3x3 matrix [j1_axis; j2_axis; j3_axis]
    % com_position: 3x1 vector (in mm)
    
    Jv = zeros(3, 3);
    
    for j = 1:3
        % For each joint, compute the vector from joint to CoM
        r = com_position - joint_positions(j,:)';
        
        % Compute Jacobian column: ω × r for revolute joints
        Jv(:, j) = cross(joint_axes(:,j), r);
    end
end