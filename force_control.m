clc; clear all; close all;

%% -------------------- DYNAMIC PROPERTIES (SI UNITS) --------------------
g = 9.81; % m/s²

% Link 1 (Base Link)
Link1.mass = 0.21025; % kg
Link1.com = [0.00512, 0.00097, 0.05928]; % m
Link1.inertia_com = [371584.58,  1133.32,   37190.55;
                     1133.32,   390041.76,  8369.32;
                     37190.55,   8369.32,   129419.43] * 1e-9; % kg·m²

% Link 2 (Middle Link)
Link2.mass = 0.11181; % kg
Link2.com = [0.00138, 0.10492, 0.00001]; % m
Link2.inertia_com = [230119.67, -26620.50,  85.99;
                    -26620.50,   53290.16,  76.49;
                     85.99,      76.49,     250996.49] * 1e-9; % kg·m²

% Link 3 (End-Effector Link)
Link3.mass = 0.03816; % kg
Link3.com = [0.01352, 0.06237, 0.00001]; % m
Link3.inertia_com = [67677.77, -487.23,  1.61;
                    -487.23,   3902,     9.04;
                     1.61,     9.04,     65335.24] * 1e-9; % kg·m²

% Test tube properties
test_tube.mass = 0.00668; % kg
test_tube.com = [0, 0, -0.04123]; % m

% Store all links
robot.links = {Link1, Link2, Link3};
robot.num_links = 3;

%% -------------------- SETUP FIGURE --------------------
figure('Renderer', 'zbuffer');
axis equal; axis vis3d; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); rotate3d on;
axis([-300 300 -300 300 -50 350]);
title("Dynamic Path Planning with Joint Torque Analysis")

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

%% -------------------- PATH GENERATION --------------------
trace_pts = struct('L1', [], 'L2', [], 'L3', [], 'Lee', []);
samples = create_samples([0,50,100], [150,150,100], [-150,100,100], 100, 8);
[num_samples, ~] = size(samples);
disp("Generated sample points")
disp(samples)
disp("--------------------------------------------------------------");
current_angle = [270,-66,156]; % Initial joint angles [deg]
r = 5;  % Target sphere radius

% Motion control parameters
velocity = 20;       % mm/s
dt = 0.1;            % Time step [s]
tolerance = 5;       % Position tolerance [mm]
max_joint_vel = 10;  % Maximum joint velocity [deg/s]
lambda = 0.1;        % Damping factor for singularity handling
singularity_threshold = 1e5;  % Condition number threshold for singularity
reachability = 347;  % Maximum radius [mm]

% Force application parameters
pickup_index = 3;          % When to pick up object
place_index = num_samples - 3; % When to place object
pickup_force = [0, 0, 1];  % N (upward during pickup)
place_force = [0, 0, -0.2];% N (downward during placement)

% Initialize data recording arrays
all_angles = [];
all_velocities = [];
all_angular_vel = [];
all_J_error = [];
all_location = [];
all_linear_vel = [];
all_torques = [];
all_gravity_torques = [];
all_ext_torques = [];
current_load = 0;          % Track if carrying object
applied_force = [0, 0, 0]; % Current external force

%% -------------------- ANIMATION & DYNAMICS LOOP --------------------
for i = 1:num_samples
    % Get current target position
    target_pos = samples(i,:);
    if norm(target_pos) > reachability
        fprintf("Out of Reach: %.4f, %.4f, %.4f\n", target_pos);
        disp("--------------------------------------------------------------")
        continue
    end

    px_target = target_pos(1);
    py_target = target_pos(2);
    pz_target = target_pos(3);

    fprintf("New target: %.2f, %.2f, %.2f mm\n", px_target, py_target, pz_target);
    
    % Create target visualization
    [bx, by, bz] = sphere(20);
    target_sphere = surf(r*bx + target_pos(1), ...
                      r*by + target_pos(2), ...
                      r*bz + target_pos(3), ...
                      'FaceColor', 'magenta', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    drawnow;
    error_norm = tolerance;
    singularity_detected = false;
    step = 0;
    
    % Update force application logic
    if i == pickup_index
        current_load = test_tube.mass;
        applied_force = pickup_force;
        fprintf("---- PICKUP EVENT: Applying +1N Z-force ----\n");
    elseif i == place_index
        current_load = 0;
        applied_force = place_force;
        fprintf("---- PLACEMENT EVENT: Applying -0.2N Z-force ----\n");
    else
        applied_force = [0, 0, 0];
    end

    % Motion control loop
    while error_norm >= tolerance
        step = step + 1;

        % Compute current end-effector position
        T_current = BaseToTool(current_angle(1), current_angle(2), current_angle(3));
        current_pos = T_current(1:3,4)';
        
        % Compute position error
        pos_error = target_pos - current_pos;
        error_norm = norm(pos_error);
        
        % Compute desired end-effector velocity
        direction = pos_error / error_norm;
        xd_dot = direction * velocity;  % Constant velocity vector
        all_linear_vel = [all_linear_vel; xd_dot];
        
        % Compute Jacobian at current configuration
        Jk = Jacobian(current_angle(1), current_angle(2), current_angle(3));
        condJ = cond(Jk);
        
        % Check for singularity
        if condJ > singularity_threshold
            fprintf("Step %d: Singularity detected (cond=%.2e). Skipping target.\n", step, condJ);
            singularity_detected = true;
            break;
        end
        
        fprintf("--------------\nStep %d: cond(J) = %.2f, Error: %.2f mm\n", step, condJ, error_norm);
        
        % Compute joint velocities using damped least squares
        J_pinv = Jk' / (Jk*Jk' + lambda^2*eye(3));
        theta_dot_rad = J_pinv * xd_dot';
        
        % Convert to deg/s and clamp velocities
        theta_dot_deg = rad2deg(theta_dot_rad)';
        theta_dot_deg = sign(theta_dot_deg) .* min(abs(theta_dot_deg), max_joint_vel);
        
        % Record current angles and velocities
        all_angles = [all_angles; current_angle];
        all_velocities = [all_velocities; theta_dot_deg];
        
        % Update joint angles
        current_angle = current_angle + theta_dot_deg * dt;
        
        % Update visualization
        theta_rad = deg2rad(current_angle);
        
        % Compute transforms
        T1 = [rotz(theta_rad(1)), [0;0;0]; 0 0 0 1];
        T2 = [roty(theta_rad(2)), d12'; 0 0 0 1];
        T3 = [roty(theta_rad(3)), d23'; 0 0 0 1];
        Tee = [eye(3), d3ee'; 0 0 0 1];

        % Transform link vertices
        V1 = apply_transform(L1.V0, T1);
        V2 = apply_transform(L2.V0, T1*T2);
        V3 = apply_transform(L3.V0, T1*T2*T3);

        set(q(1), 'Vertices', V1');
        set(q(2), 'Vertices', V2');
        set(q(3), 'Vertices', V3');

        % Update joint positions for tracing
        joint1 = T1;
        joint1_axis = joint1(1:3,3);
        joint1_pos = T1(1:3,4)';

        joint2 = T1*T2;
        joint2_axis = joint2(1:3,2);
        joint2_pos = joint2(1:3,4)';

        joint3 = joint2*T3;
        joint3_axis = joint3(1:3,2);
        joint3_pos = joint3(1:3,4)';

        ee = joint3*Tee;
        ee_pos = ee(1:3,4)';
        all_location = [all_location; ee_pos];

        % Angular velocity
        J_omega = [joint1_axis, joint2_axis, joint3_axis];
        angle_vel = (J_omega * theta_rad')';
        all_angular_vel = [all_angular_vel; angle_vel];

        % Check the Jacobian matrix with cross product
        JP1 = cross(joint1_axis, (ee_pos - joint1_pos)');
        JP2 = cross(joint2_axis, (ee_pos - joint2_pos)');
        JP3 = cross(joint3_axis, (ee_pos - joint3_pos)');
        J_cross = [JP1, JP2, JP3];
        
        diff_J = J_cross - Jk;
        norm_diff = norm(diff_J, 'fro');
        norm_Jk = norm(Jk, 'fro');
        norm_J_cross = norm(J_cross, 'fro');
        denom = (norm_Jk + norm_J_cross) / 2;
        rel_norm_error = norm_diff / (denom + eps);
        all_J_error = [all_J_error, rel_norm_error];

        % ==================== DYNAMICS ANALYSIS ====================
        % Convert Jacobian to meters (from mm to m)
        J_ee_m = Jk / 1000;
        
        % Compute torque due to external force
        ext_torque = J_ee_m' * applied_force';
        
        % Compute gravity compensation torque
        gravity_torque = zeros(3,1);
        for link_idx = 1:robot.num_links
            % Gravity force vector for this link
            gravity_force = [0; 0; -robot.links{link_idx}.mass * g];
            
            % Simplified COM Jacobian (approximation)
            J_com = compute_com_jacobian(theta_rad, link_idx);
            
            % Add to total gravity torque
            gravity_torque = gravity_torque + J_com' * gravity_force;
        end
        
        % Add torque due to test tube if carried
        if current_load > 0
            tube_gravity_force = [0; 0; -current_load * g];
            gravity_torque = gravity_torque + J_ee_m' * tube_gravity_force;
        end
        
        % Total torque = gravity compensation + external forces
        total_torque = gravity_torque + ext_torque;
        
        % Store torque values
        all_torques = [all_torques; total_torque'];
        all_gravity_torques = [all_gravity_torques; gravity_torque'];
        all_ext_torques = [all_ext_torques; ext_torque'];
        % ==========================================================
        
        % Append to trace points
        trace_pts.L1 = [trace_pts.L1; joint1_pos];
        trace_pts.L2 = [trace_pts.L2; joint2_pos];
        trace_pts.L3 = [trace_pts.L3; joint3_pos];
        trace_pts.Lee = [trace_pts.Lee; ee_pos];

        % Draw traces
        plot3(trace_pts.L1(:,1), trace_pts.L1(:,2), trace_pts.L1(:,3), 'r.', 'MarkerSize', 1);
        plot3(trace_pts.L2(:,1), trace_pts.L2(:,2), trace_pts.L2(:,3), 'g.', 'MarkerSize', 1);
        plot3(trace_pts.L3(:,1), trace_pts.L3(:,2), trace_pts.L3(:,3), 'b.', 'MarkerSize', 1);
        plot3(trace_pts.Lee(:,1), trace_pts.Lee(:,2), trace_pts.Lee(:,3), 'b.', 'MarkerSize', 1);
        
        % Draw coordinate frames
        draw_frame(T1, 30);
        draw_frame(joint2, 30);
        draw_frame(joint3, 30);
        draw_frame(ee, 30);
        
        drawnow;
        
        % Print current status
        fprintf("Angle Configuration (deg): %.4f, %.4f, %.4f\n", current_angle);
        fprintf("Location (mm): %.4f, %.4f, %.4f\n", ee_pos);
        disp("Geometric (cross product) Jacobian matrix")
        disp(J_cross);
        disp("Analytical (derivative) Jacobian matrix")
        disp(Jk)
    end
    
    % Record final state for this target
    all_angles = [all_angles; current_angle];
    all_velocities = [all_velocities; [0, 0, 0]];
    all_angular_vel = [all_angular_vel; [0,0,0]];
    all_J_error = [all_J_error, all_J_error(end)];
    all_location = [all_location; all_location(end, :)];
    all_linear_vel = [all_linear_vel; [0,0,0]];
    
    % Skip to next sample if singularity detected
    if singularity_detected
        fprintf("Skipping to next target due to singularity\n");
        disp("--------------------------------------------------------------");
        continue;
    end
    
    disp("--------------------------------------------------------------");
end

%% -------------------- PLOT JOINT DATA --------------------
% Calculate angular acceleration using central differences with zero padding
acceleration_data = [[0,0,0]; all_velocities; [0,0,0]];  % Pad velocities with zeros at both ends

% Compute acceleration using central differences
n = size(all_velocities, 1);
acceleration = zeros(n, 3);  % Preallocate acceleration array

for j = 1:3  % For each joint
    for i = 1:n  % For each time point
        % Central difference: (f(i+1) - f(i-1)) / (2*dt)
        acceleration(i, j) = (acceleration_data(i+2, j) - acceleration_data(i, j)) / (2*dt);
    end
end

% Create time vector for plotting
time_vector = (0:length(all_angles)-1)*dt;

% Plot joint motion profiles
joint_names = {'Joint 1', 'Joint 2', 'Joint 3'};
figure;

for j = 1:3
    subplot(3, 3, j);
    plot(time_vector, all_angles(:, j), 'LineWidth', 2);
    title([joint_names{j} ' - Displacement']);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    grid on;
    
    subplot(3, 3, j+3);
    plot(time_vector, all_velocities(:, j), 'LineWidth', 2);
    title([joint_names{j} ' - Velocity']);
    xlabel('Time (s)');
    ylabel('Velocity (deg/s)');
    grid on;
    
    subplot(3, 3, j+6);
    plot(time_vector, acceleration(:, j), 'LineWidth', 2);
    title([joint_names{j} ' - Acceleration']);
    xlabel('Time (s)');
    ylabel('Acceleration (deg/s^2)');
    grid on;
end
sgtitle('Joint Motion Profiles');

figure;
plot(time_vector, all_J_error, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Relative Frobenius Error');
title('Jacobian Discrepancy');
grid on;

figure;
axis_name = {'\alpha', '\beta', '\gamma'};
for i = 1:3
    subplot(3, 3, i);
    plot(time_vector, all_angular_vel(:, i), 'LineWidth', 2);
    title([axis_name{i} ' - Angular Velocity']);
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    grid on;

    subplot(3, 3, i+3);
    plot(time_vector, all_linear_vel(:, i), 'LineWidth', 2);
    title([axis_name{i} ' - Linear Velocity']);
    xlabel('Time (s)');
    ylabel('Linear Velocity (mm/s)');
    grid on;

    subplot(3, 3, i+6);
    plot(time_vector, all_location(:, i), 'LineWidth', 2);
    title([axis_name{i} ' - Spatial Position']);
    xlabel('Time (s)');
    ylabel('Location (mm)');
    grid on;
end
sgtitle('End Effector Motion Profile');

%% -------------------- PLOT TORQUE ANALYSIS --------------------
time_vector_torque = (0:size(all_torques,1)-1)*dt;

% Plot total joint torques
figure;
subplot(3,1,1);
plot(time_vector_torque, all_torques(:,1), 'LineWidth', 2);
title('Joint 1 Torque');
ylabel('Torque (N·m)');
grid on;

subplot(3,1,2);
plot(time_vector_torque, all_torques(:,2), 'LineWidth', 2);
title('Joint 2 Torque');
ylabel('Torque (N·m)');
grid on;

subplot(3,1,3);
plot(time_vector_torque, all_torques(:,3), 'LineWidth', 2);
title('Joint 3 Torque');
xlabel('Time (s)');
ylabel('Torque (N·m)');
grid on;
sgtitle('Joint Torques During Motion');

% Plot torque components for each joint
joint_names = {'Joint 1', 'Joint 2', 'Joint 3'};
for joint = 1:3
    figure;
    hold on;
    plot(time_vector_torque, all_gravity_torques(:,joint), 'b-', 'LineWidth', 1.5);
    plot(time_vector_torque, all_ext_torques(:,joint), 'r-', 'LineWidth', 1.5);
    plot(time_vector_torque, all_torques(:,joint), 'k--', 'LineWidth', 2);
    
    xlabel('Time (s)');
    ylabel('Torque (N·m)');
    title([joint_names{joint} ' Torque Components']);
    legend('Gravity', 'External Forces', 'Total');
    grid on;
    hold off;
end

%% -------------------- HELPER FUNCTIONS --------------------
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

function samples_path = create_samples(knot_point, start_of_arc_point, end_of_arc_point, drop_point_z_offset, N_arc_samples)
    % Define radius boundaries
    r_min = 28;
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

function J_com = compute_com_jacobian(theta_rad, link_idx)
    % Simplified COM Jacobian calculation based on joint configuration
    % For accurate industrial use, replace with full geometric Jacobian
    switch link_idx
        case 1  % Base link
            J_com = [0, 0, 0;
                     0, 0, 0;
                     0, 0, 0];
            
        case 2  % Middle link
            J_com = [-0.1*sin(theta_rad(1)), 0, 0;
                      0.1*cos(theta_rad(1)), 0, 0;
                      0, 0, 0];
            
        case 3  % End-effector link
            J_com = [-0.25*sin(theta_rad(1)), -0.15*sin(theta_rad(1)), -0.1*sin(theta_rad(1));
                      0.25*cos(theta_rad(1)),  0.15*cos(theta_rad(1)),  0.1*cos(theta_rad(1));
                      0, 0, 0];
    end
end