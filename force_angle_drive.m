close all; clc; clear all;

%% -------------------- DYNAMIC PROPERTIES (SI UNITS) --------------------
g = 9.81; % m/s^2

% Link 1 (Base Link)
Link1.mass = 204.72 * 1e-3; % kg
Link1.com_1 = [1.77; -4.90; 59.45] * 1e-3; % m

% Link 2 (Middle Link)
Link2.mass = 108.74 * 1e-3; % kg
Link2.com_2 = [105.48; 11.41; -0.21] * 1e-3; % m

% Link 3 (End-Effector Link)
Link3.mass = 36.51 * 1e-3; % kg
Link3.com_3 = [62.65; -1.95; -0.01] * 1e-3; % m

% Test tube properties
test_tube.mass = 0.035; % kg
test_tube.com = [0; 0; -0.04165]; % m (relative to end-effector)

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
samples = create_samples([50,0,100], [150,150,100], [150,-100,100], 100, 5);
[num_samples, ~] = size(samples);
disp("Generated sample points")
disp(samples)
disp("--------------------------------------------------------------");
r = 5;

% Motion parameters
dt = 0.1;            % Time step [s]
angle_constaints = [-300 -145 -250; 300 145 250];
constraints = deg2rad(angle_constaints);
position_tolerance = 1;  % 1 mm position tolerance
current_angle = [270,-66,156];  % degrees

% Force application parameters
pickup_index = 4;          % When to pick up object
place_index = num_samples - 2; % When to place object
force_vector = test_tube.com * test_tube.mass .* [0; 0; -g];
pickup_force = force_vector * 1.3;  % N (upward during pickup)
place_force = force_vector * 0.7;% N (downward during placement)
hold_force = force_vector; % N (upward during hold)

% Initialize data recording arrays
all_times = [];          % Put time stamp
all_angles = [];         % Put current angles
all_angular_vel = [];    % Put angle_vel of EE
all_J_error = [];        % Put similarity score J
all_location = [];       % Put x, y, z of EE
all_linear_vel = [];     % Put linear_vel of EE
all_torques = [];
all_gravity_torques = [];
all_ext_torques = [];
velocity = [];
acceleration = [];
all_steps = [];            % Put step number
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
        applied_force = [0; 0; 0];
    end
    
    % ========== VELOCITY CONTROL ========== %
    theta = []; theta_dot = []; theta_ddot = []; steps = 1;
    time_vec = [];
    % Generate cubic trajectory
    for joint = 1:3
        [d_deg, v_deg, a_deg, t_seg] = interpolate(...
            current_angle(joint), best_solution(joint), 4, dt);
        theta = [theta; d_deg];
        theta_dot = [theta_dot; v_deg];
        theta_ddot = [theta_ddot; a_deg];
        time_vec = t_seg;
    end

    all_angles = [all_angles; theta'];
    velocity = [velocity; theta_dot'];
    acceleration = [acceleration; theta_ddot'];
    all_times = [all_times, t_seg + 4*i];
    steps = length(t_seg);
    all_steps = [all_steps, steps];
    
    current_angle = best_solution;

    theta = deg2rad(theta);
    theta_dot = deg2rad(theta_dot);
    for k = 1:steps
        theta_rad = theta(:, k)';
        current_angles_deg = rad2deg(theta_rad);
        theta_dot_rad = theta_dot(:,k);
        
        % Update visualization
        T1 = [rotz(theta(1,k)), [0;0;0]; 0 0 0 1];
        T2 = [roty(theta(2,k)), d12'; 0 0 0 1];
        T3 = [roty(theta(3,k)), d23'; 0 0 0 1];
        Tee = [[1,0,0;0,1,0;0,0,1], d3ee'; 0 0 0 1];
        
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
        all_location = [all_location; jointee_pos];

        V1 = apply_transform(L1.V0, joint1);
        V2 = apply_transform(L2.V0, joint2);
        V3 = apply_transform(L3.V0, joint3);

        set(q(1), 'Vertices', V1');
        set(q(2), 'Vertices', V2');
        set(q(3), 'Vertices', V3');

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
        all_J_error = [all_J_error; rel_norm_error];

        J_full = [J_cross; [z1, z2, z3]];
        wrench_vec = J_full * theta_dot_rad;
        all_linear_vel = [all_linear_vel; wrench_vec(1:3)'];
        all_angular_vel = [all_angular_vel; wrench_vec(4:6)'];

        % ==================== DYNAMICS ANALYSIS ====================
        % Transform CoMs to base frame (in meters)
        com1_base = joint1(1:3, 1:3) * Link1.com_1; 
        
        com2_base = joint2(1:3, 1:3) * Link2.com_2;
        
        com3_base = joint3(1:3, 1:3) * Link3.com_3;
        
        position_bundle = [joint1_pos; joint2_pos; joint3_pos]./1000;
        axis_bundle = [z1, z2, z3];
        
        % Compute CoM Jacobians
        J_com1 = compute_com_jacobian(position_bundle, axis_bundle, com1_base);
        J_com2 = compute_com_jacobian(position_bundle, axis_bundle, com2_base);
        J_com3 = compute_com_jacobian(position_bundle, axis_bundle, com3_base);
        
        % Gravity torque calculation
        g_vec = [0; 0; -g];
        gravity_torque = ...
            J_com1' * (Link1.mass * g_vec) + ...
            J_com2' * (Link2.mass * g_vec) + ...
            J_com3' * (Link3.mass * g_vec);
        
        % Test tube handling
        if current_load > 0
            tube_com_base = jointee(1:3, 1:3) * test_tube.com;
            J_tube = compute_com_jacobian(position_bundle, axis_bundle, tube_com_base);
            gravity_torque = gravity_torque + J_tube' * (current_load * g_vec);
        end
        
        % External force torque
        ext_torque = Jk' * applied_force;
        
        % Total torque
        total_torque = gravity_torque + ext_torque;
        
        % Store torque values
        all_torques = [all_torques; total_torque'];
        all_gravity_torques = [all_gravity_torques; gravity_torque'];
        all_ext_torques = [all_ext_torques; ext_torque'];
    end

    disp("--------------------------------------------------------------");
end

%% -------------------- PLOT JOINT DATA --------------------
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
    xline(all_times(sum(all_steps(1: pickup_index-1))), 'g--', 'Pickup', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xline(all_times(sum(all_steps(1: place_index-1))), 'm--', 'Placement', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Angular Velocity
    subplot(3,3,j+3);
    plot(all_times, all_ext_torques(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - External Forces']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    grid on;
    xline(all_times(sum(all_steps(1: pickup_index-1))), 'g--', 'Pickup', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xline(all_times(sum(all_steps(1: place_index-1))), 'm--', 'Placement', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Angular Acceleration
    subplot(3,3,j+6);
    plot(all_times, all_torques(:,j), 'LineWidth', 2);
    title([joint_names{j} ' - Total Torques']);
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    grid on;
    xline(all_times(sum(all_steps(1: pickup_index-1))), 'g--', 'Pickup', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xline(all_times(sum(all_steps(1: place_index-1))), 'm--', 'Placement', 'LineWidth', 1.5, 'HandleVisibility', 'off');
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

function Jv = compute_com_jacobian(joint_positions, joint_axes, com_position)
    % joint_positions: 3x3 matrix [j1_pos; j2_pos; j3_pos] (in m)
    % joint_axes: 3x3 matrix [j1_axis; j2_axis; j3_axis]
    % com_position: 3x1 vector (in m)
    
    Jv = zeros(3, 3);
    
    for j = 1:3
        r = com_position - joint_positions(j,:)';
        Jv(:, j) = cross(joint_axes(:,j), r);
    end
end

function [d,v,a,t] = interpolate(theta_0,theta_f,tf,step)
    % [d,v,a,t] = cubic_scheme(theta_0,theta_f,tf,step);
    [d,v,a,t] = quintic_scheme(theta_0,theta_f,tf,step);
end

function [d,v,a,t]=quintic_scheme(theta_0,theta_f,tf,step)
    d =[];
    v = [];
    a = [];
    n=length(theta_0);
    for i=1:n
        % Determning the value of the coefficients for each joint
        a0=theta_0(i);
        a1=0;
        a2=0;
        a3=(20*theta_f(i)-20*theta_0(i))/(2*tf^3);
        a4=(30*theta_0(i)-30*theta_f(i))/(2*tf^4);
        a5=(12*theta_f(i)-12*theta_0(i))/(2*tf^5);
        
        % Identifying the values of displacement, velocity, and acceleration of the joints
        t=0:step:tf;
        d=[d;a0+a1.*t+a2.*t.^2+a3.*t.^3+a4.*t.^4+a5.*t.^5];
        v=[v;a1+2.*a2.*t+3.*a3.*t.^2+4.*a4.*t.^3+5.*a5.*t.^4];
        a=[a;2.*a2+6.*a3.*t+12.*a4.*t.^2+20.*a5.*t.^3];
    end
end

function [d,v,a,t]=cubic_scheme(theta_0,theta_f,tf,step)
    d=[];
    v=[];
    a=[];
    n=length(theta_0);
    for i=1:n
        % Determning the value of the coefficients for each joint
        a0=theta_0(i);
        a1=0;
        a2=3/tf^2*(theta_f(i)-theta_0(i));
        a3=-2/tf^3*(theta_f(i)-theta_0(i));
        % Identifying the values of displacement, velocity, and acceleration of the joints
        t=0:step:tf;
        d=[d;a0+a1.*t+a2.*t.^2+a3.*t.^3];
        v=[v;a1+2.*a2.*t+3.*a3.*t.^2];
        a=[a;2.*a2+6.*a3.*t];
    end
end
