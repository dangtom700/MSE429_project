close all; clc; clear all;

%% -------------------- SETUP FIGURE --------------------
figure('Renderer', 'zbuffer');
axis equal; axis vis3d; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); rotate3d on;
axis([-300 300 -300 300 -200 350]);
title("Path Planning Using Jacobian Joint-Space Trajectory")

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
num_samples = 100;
samples = create_samples(num_samples);
current_angle = [0, 0, 90];  % Initial joint angles [deg]
r = 5;  % Target sphere radius

% Motion control parameters
velocity = 20;       % End-effector speed [mm/s]
dt = 0.1;            % Time step [s]
tolerance = 5;       % Position tolerance [mm]
max_joint_vel = 30;  % Maximum joint velocity [deg/s]
lambda = 0.1;        % Damping factor for singularity handling
max_steps = 100;     % Maximum steps per target
singularity_threshold = 1e5;  % Condition number threshold for singularity
reachability = 250;  % Maximum radius

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
    step = 0;
    singularity_detected = false;
    
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
        
        % Compute Jacobian at current configuration
        Jk = Jacobian(current_angle(1), current_angle(2), current_angle(3));
        condJ = cond(Jk);
        
        % Check for singularity
        if condJ > singularity_threshold
            fprintf("Step %d: Singularity detected (cond=%.2e). Skipping target.\n", step, condJ);
            singularity_detected = true;
            break;
        end
        
        fprintf("Step %d: cond(J) = %.2f, Error: %.2f mm\n", step, condJ, error_norm);
        
        % Compute joint velocities using damped least squares
        J_pinv = Jk' / (Jk*Jk' + lambda^2*eye(3));
        theta_dot_rad = J_pinv * xd_dot';
        
        % Convert to deg/s and clamp velocities
        theta_dot_deg = rad2deg(theta_dot_rad)';
        theta_dot_deg = sign(theta_dot_deg) .* min(abs(theta_dot_deg), max_joint_vel);
        
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
        joint1_pos = T1(1:3,4)';
        joint2 = T1*T2;
        joint2_pos = joint2(1:3,4)';
        joint3 = joint2*T3;
        joint3_pos = joint3(1:3,4)';
        ee = joint3*Tee;
        ee_pos = ee(1:3,4)';

        % Append to trace points
        trace_pts.L1 = [trace_pts.L1; joint1_pos];
        trace_pts.L2 = [trace_pts.L2; joint2_pos];
        trace_pts.L3 = [trace_pts.L3; joint3_pos];
        trace_pts.Lee = [trace_pts.Lee; ee_pos];

        % % Draw traces
        % plot3(trace_pts.L1(:,1), trace_pts.L1(:,2), trace_pts.L1(:,3), 'r.', 'MarkerSize', 1);
        % plot3(trace_pts.L2(:,1), trace_pts.L2(:,2), trace_pts.L2(:,3), 'g.', 'MarkerSize', 1);
        % plot3(trace_pts.L3(:,1), trace_pts.L3(:,2), trace_pts.L3(:,3), 'b.', 'MarkerSize', 1);
        plot3(trace_pts.Lee(:,1), trace_pts.Lee(:,2), trace_pts.Lee(:,3), 'b.', 'MarkerSize', 1);

        % % Draw coordinate frames
        % draw_frame(T1, 30);
        % draw_frame(joint2, 30);
        % draw_frame(joint3, 30);
        % draw_frame(ee, 30);
        
        drawnow;
        
        % Print current status
        fprintf("Angles: [%.1f, %.1f, %.1f] deg | ", current_angle);
        fprintf("Position: [%.1f, %.1f, %.1f] mm\n", ee_pos);
    end
    
    % Remove target visualization
    delete(target_sphere);
    
    % Skip to next sample if singularity detected
    if singularity_detected
        fprintf("Skipping to next target due to singularity\n");
        disp("--------------------------------------------------------------");
        continue;
    end
    
    disp("--------------------------------------------------------------");
end

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

function samples = create_samples(num_samples)
    r_max = 300;
    r_min_xy = 50;

    samples = [];

    while size(samples, 1) < num_samples
        % Sample in a cube, filter later
        x = (2 * rand(num_samples, 1) - 1) * r_max;  % x ∈ [-300, 300]
        y = (2 * rand(num_samples, 1) - 1) * r_max;  % y ∈ [-300, 300]
        z = rand(num_samples, 1) * r_max;            % z ∈ [0, 300]

        r = sqrt(x.^2 + y.^2 + z.^2);
        % Keep points inside a hemisphere with abs(x), abs(y) ≥ 90
        valid = (r <= r_max) & (z > 0) & ...
                (abs(x) >= r_min_xy) & (abs(y) >= r_min_xy);

        % Append valid samples
        samples = [samples; [x(valid), y(valid), z(valid)]];
    end

    % Trim to exact number
    samples = samples(1:num_samples, :);
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

function distance_gain = Jacobian(theta1_deg, theta2_deg, theta3_deg)
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
       
    distance_gain = J;
end