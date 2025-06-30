close all; clc;

%% -------------------- SETUP FIGURE --------------------
figure('Renderer', 'zbuffer');
axis equal; axis vis3d; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); rotate3d on;
axis([-300 300 -300 300 -200 350]);

% Lighting setup
light('Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;

%% -------------------- DEFINE COLORS --------------------
link_colors  = {[0.6 0.6 0.4], [0.4 0.6 0.8], [0.8 0.4 0.6]};
frame_colors = {'r', 'g', 'b'};

%% -------------------- JOINT & LINK SETUP --------------------
% Home positions (relative to base)
joint1 = [0, 0, 0];
joint2 = [1.3, 40, 95];
joint3 = [-132, 12.5, 95.5];

% Relative translations
d12 = joint2 - joint1;
d23 = joint3 - joint2;

% Load STL models and align
[L1.F, L1.V] = load_link('Link1.STL', [97.9527, 34.4903, 134.099], eye(3));
[L2.F, L2.V] = load_link('Link2.STL', [157.325, 42.4651, 173.659], [1 0 0; 0 0 -1; 0 1 0]);
[L3.F, L3.V] = load_link('Link3.STL', [127.04, 113.902, 187.562], [1 0 0; 0 0 -1; 0 1 0]);

% Transpose vertices
L1.V0 = L1.V';
L2.V0 = L2.V';
L3.V0 = L3.V';

% Create patch objects
q(1) = patch('Faces', L1.F, 'Vertices', L1.V0', 'FaceColor', link_colors{1}, 'EdgeColor', 'none');
q(2) = patch('Faces', L2.F, 'Vertices', L2.V0', 'FaceColor', link_colors{2}, 'EdgeColor', 'none');
q(3) = patch('Faces', L3.F, 'Vertices', L3.V0', 'FaceColor', link_colors{3}, 'EdgeColor', 'none');

%% -------------------- ANIMATION --------------------
trace_pts = struct('L1', [], 'L2', [], 'L3', []);
num_samples = 10;
samples = create_samples(num_samples);  % You must define this function
steps = 10;
current_angle = [0, 0, -90];  % Degrees
r = 5;  % Radius of the ball

for i = 1:num_samples
    % Define your target for IK (example placeholders)
    px_target = samples(i,1);
    py_target = samples(i,2);
    pz_target = samples(i,3);

    % Draw a sphere at the target position
    [bx, by, bz] = sphere(20);  % 20x20 sphere resolution
    sx = r * bx - px_target;
    sy = r * by - py_target;
    sz = r * bz + pz_target + 95;

    fprintf("New location target: %.4f,%.4f,%.4f\n", px_target, py_target, pz_target);

    % Get all inverse kinematics solutions
    all_solutions = inverseKinematics(px_target, py_target, pz_target);
    best_solution = [];
    min_error = Inf;

    for j = 1:size(all_solutions, 1)
        sol_deg = rad2deg(all_solutions(j,:));
        T = BaseToEnd(sol_deg(1), sol_deg(2), sol_deg(3));
        position_error = norm([px_target, py_target, pz_target] - T(1:3,4)');
        angle_error = norm(wrapTo180(current_angle - sol_deg));

        if position_error < min_error
            min_error = position_error;
            best_solution = sol_deg;
        end
    end

    if isempty(best_solution)
        disp("There is no solution for the new solution. Skip to the next available point.")
        disp("--------------------------------------------------------------");
        continue
    end
    fprintf("Error: %.4f\nTarget angle (rad): %.4f,%.4f,%.4f\n", min_error, best_solution(1), best_solution(2), best_solution(3));

    % Plot the target sphere
    target_ball = surf(sx, sy, sz, ...
        'FaceColor', 'magenta', 'EdgeColor', 'none', ...
        'FaceAlpha', 0.3);  % Transparent magenta sphere
    
    % Ensure the ball is rendered before motion begins
    drawnow;

    % Interpolate motion
    % Ensure shortest angle path by wrapping delta to [-180, 180]
    delta_angle = wrapTo180(best_solution - current_angle);  % In degrees
    
    % Angular velocity per step
    vel = delta_angle / steps;
    
    % Preallocate angle trajectory (in radians)
    theta = zeros(3, steps);
    
    % Integrate angles step by step
    for s = 1:steps
        step_angle = current_angle + vel * s;  % Step in degrees
        theta(:,s) = deg2rad(step_angle);      % Convert to radians
    end
    current_angle = best_solution;

    for k = 1:steps
        % Compute transformations
        T1 = [rotz(rad2deg(theta(1,k))), [0;0;0]; 0 0 0 1];
        T2 = [roty(rad2deg(theta(2,k))), d12'; 0 0 0 1];
        T3 = [roty(rad2deg(theta(3,k))), d23'; 0 0 0 1];

        % Apply transforms to link vertices
        V1 = apply_transform(L1.V0, T1);
        V2 = apply_transform(L2.V0, T1*T2);
        V3 = apply_transform(L3.V0, T1*T2*T3);

        set(q(1), 'Vertices', V1');
        set(q(2), 'Vertices', V2');
        set(q(3), 'Vertices', V3');

        % Joint positions
        joint1 = T1;
        joint1_pos = T1(1:3,4)';
        joint2 = (T1 * T2);
        joint2_pos = joint2(1:3,4)';
        joint3 = (T1 * T2 * T3);
        joint3_pos = joint3(1:3,4)';

        trace_pts.L1 = [trace_pts.L1; joint1_pos];
        trace_pts.L2 = [trace_pts.L2; joint2_pos];
        trace_pts.L3 = [trace_pts.L3; joint3_pos];

        % Plot trajectories
        plot3(trace_pts.L1(:,1), trace_pts.L1(:,2), trace_pts.L1(:,3), 'r.', 'MarkerSize', 1);
        plot3(trace_pts.L2(:,1), trace_pts.L2(:,2), trace_pts.L2(:,3), 'g.', 'MarkerSize', 1);
        plot3(trace_pts.L3(:,1), trace_pts.L3(:,2), trace_pts.L3(:,3), 'b.', 'MarkerSize', 1);

        % Draw coordinate frames
        draw_frame(joint1, 30);
        draw_frame(joint2, 30);
        draw_frame(joint3, 30);
        drawnow;
        
        % Get location of end point
        degree_angle = rad2deg(theta(:,k))';
        ee = BaseToEnd(degree_angle(1), degree_angle(2), degree_angle(3));
        fprintf("Angle Configuration (rad): %.4f,%.4f,%.4f\n", degree_angle(1), degree_angle(2), degree_angle(3));
        fprintf("Location (mm): %.4f,%.4f,%.4f\n", ee(1,4), ee(2,4), ee(3,4));
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

function R = rotz(theta)
    c = cosd(theta); s = sind(theta);
    R = [c -s 0; s c 0; 0 0 1];
end

function R = roty(theta)
    c = cosd(theta); s = sind(theta);
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

function samples = create_samples(num_samples)
    % Define Cartesian bounds
    x_min = 0; x_max = 305;
    y_min = 0; y_max = 305;
    z_min = 0;  z_max = 360;

    samples = [];
    while size(samples, 1) < num_samples
        % Generate candidate points
        x = x_min + (x_max - x_min) * rand(num_samples, 1);
        y = y_min + (y_max - y_min) * rand(num_samples, 1);
        z = z_min + (z_max - z_min) * rand(num_samples, 1);

        % Spherical filtering
        r = sqrt(x.^2 + y.^2 + z.^2);
        valid = (r >= 91.85) & (r <= 368.02);
        samples = [samples; [x(valid), y(valid), z(valid)]];

        % Limit to requested number
        samples = samples(1:min(end,num_samples), :);
    end
end

function solutions = inverseKinematics(px, py, pz)
    % Link parameters (mm)
    A = 6.78;         % a1
    B = 133.794;      % a2
    C = 112.268;      % a3 (end effector)
    
    solutions = [];   % Initialize solution matrix
    
    % ===== Step 1: Solve for θ₁ =====
    theta1_rad = atan2(py, px);
    
    % Adjust θ₁ to [0°, 360°) range
    if theta1_rad < 0
        theta1_rad = theta1_rad + 2*pi;
    end
    
    % ===== Step 2: Solve for θ₂ and θ₃ =====
    r = sqrt(px^2 + py^2);  % Radial distance in XY-plane
    r_prime = r - A;
    
    % Intermediate terms
    K = (r_prime^2 + pz^2 + B^2 - C^2) / (2 * B);
    R = sqrt(r_prime^2 + pz^2);
    
    % Check reachability
    if abs(K) > R
        return;  % Point outside workspace
    end
    
    % Base angle for θ₂ solutions
    alpha = atan2(pz, r_prime);
    beta = acos(K / R);
    
    % Two candidate solutions for θ₂ (elbow up/down)
    theta2_rad_candidates = [alpha + beta; alpha - beta];
    
    for k = 1:2
        theta2_rad = theta2_rad_candidates(k);
        
        % Solve for θ₃
        cos_phi = (r_prime - B * cos(theta2_rad)) / C;
        sin_phi = (pz - B * sin(theta2_rad)) / C;
        
        % Prevent numerical errors
        cos_phi = max(min(cos_phi, 1), -1);
        sin_phi = max(min(sin_phi, 1), -1);
        
        phi_rad = atan2(sin_phi, cos_phi);  % θ₂ + θ₃
        theta3_rad = phi_rad - theta2_rad;
        
        % Store valid solution
        solutions = [solutions; theta1_rad, theta2_rad, theta3_rad];
    end
end

function T = BaseToEnd(t1, t2, t3)
    c1 = cosd(t1); s1 = sind(t1); c2=cosd(t2); s2=sind(t2);
    c23 = cosd(t2 + t3); s23 = sind(t2 + t3);
    T = [c1*c23, -c1*s23, s1, c1*(6.78 + 133.784*c2 + 112.268*c23);
         s1*c23, -s1*s23, -c1, s1*(6.78 + 133.784*c2 + 112.268*c23);
         s23, c23, 1, 133.794*s2 + 112.268*s23;
         0,0,0,1];
end