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
steps = 20;
current_angle = [0, 0, 90];  % degrees
r = 5;

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

    for j = 1:size(all_solutions, 1)
        sol_deg = rad2deg(all_solutions(j,:));
    
        % Filter BEFORE using
        sol_deg = filter_angle([-130 -90 -90; 130 90 90], sol_deg);
        if isempty(sol_deg)
            continue;  % Skip this solution
        end
    
        T = BaseToEnd(sol_deg(1), sol_deg(2), sol_deg(3));
        coordinate_error = [px_target, py_target, pz_target] - T(1:3,4)';
        position_error = norm(coordinate_error);
        angle_error = norm(wrapTo180(current_angle - sol_deg));
    
        if position_error < min_error
            min_coordinate_error = coordinate_error;
            min_error = position_error;
            best_solution = sol_deg;
        end
    end

    if isempty(best_solution)
        disp("There is no solution for the new location. Skip to the next available point.")
        disp("--------------------------------------------------------------");
        continue
    end
    fprintf("Distance error: %.4f\n", min_error);
    fprintf("Target angle (deg): %.4f,%.4f,%.4f\n", best_solution(1), best_solution(2), best_solution(3));
    fprintf("Coordinate error (mm): %.4f,%.4f,%.4f\n", min_coordinate_error(1), min_coordinate_error(2), min_coordinate_error(3));

    target_ball = surf(sx, sy, sz, 'FaceColor', 'magenta', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    drawnow;
    
    % Velocity calculation
    delta_angle = wrapTo180(best_solution - current_angle);
    vel = delta_angle / steps;
    theta = zeros(3, steps);

    for s = 1:steps
        step_angle = current_angle + vel * s;
        theta(:,s) = deg2rad(step_angle);
    end
    current_angle = best_solution;

    for k = 1:steps
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

        joint1 = T1;
        joint1_pos = T1(1:3,4)';
        joint2 = joint1 * T2;
        joint2_pos = joint2(1:3,4)';
        joint3 = joint2 * T3;
        joint3_pos = joint3(1:3,4)';
        jointee = joint3 * Tee;
        jointee_pos = jointee(1:3,4)';

        trace_pts.L1 = [trace_pts.L1; joint1_pos];
        trace_pts.L2 = [trace_pts.L2; joint2_pos];
        trace_pts.L3 = [trace_pts.L3; joint3_pos];
        trace_pts.Lee = [trace_pts.Lee; jointee_pos];

        plot3(trace_pts.L1(:,1), trace_pts.L1(:,2), trace_pts.L1(:,3), 'r.', 'MarkerSize', 1);
        plot3(trace_pts.L2(:,1), trace_pts.L2(:,2), trace_pts.L2(:,3), 'g.', 'MarkerSize', 1);
        plot3(trace_pts.L3(:,1), trace_pts.L3(:,2), trace_pts.L3(:,3), 'b.', 'MarkerSize', 1);
        plot3(trace_pts.Lee(:,1), trace_pts.Lee(:,2), trace_pts.Lee(:,3), 'b.', 'MarkerSize', 1);

        %draw_frame(joint1, 30);
        %draw_frame(joint2, 30);
        %draw_frame(joint3, 30);
        draw_frame(jointee, 30);
        drawnow;

        degree_angle = rad2deg(theta(:,k))';
        fprintf("Angle Configuration (deg): %.4f, %.4f, %.4f\n", degree_angle(1), degree_angle(2), degree_angle(3));
        fprintf("Location (mm): %.4f, %.4f, %.4f\n", jointee_pos(1), jointee_pos(2), jointee_pos(3));

        ditance_correction = sample - jointee_pos;
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

function T = BaseToEnd(theta1_deg, theta2_deg, theta3_deg)
    theta1 = deg2rad(theta1_deg);
    theta2 = deg2rad(theta2_deg);
    theta3 = deg2rad(theta3_deg);

    a = cos(theta1); b = sin(theta1);
    c = cos(theta2); d = sin(theta2);
    f = cos(theta3); g = sin(theta3);

    cf_df = c*f - d*f;
    cg_dg = c*g + d*g;
    c_plus_d = c + d;
    c_minus_d = c - d;

    R = [a*cf_df,  -b,     a*cg_dg;
         b*cf_df,   a,     b*cg_dg;
         -f*c_plus_d, 0,   g*c_minus_d];

    Px = a*(1.3 - 133.3*c + 0.5*d - 126.994*cf_df + 2.8614*cg_dg) - 0.26451*b;
    Py = b*(1.3 - 133.3*c + 0.5*d - 126.994*cf_df + 2.8614*cg_dg) + 0.26451*a;
    Pz = 95 + 133.3*d + 0.5*c + 126.994*f*c_plus_d + 2.8614*g*c_minus_d;

    T = [R, [Px; Py; Pz]; [0, 0, 0, 1]];
end

function solutions = inverseKinematics(px, py, pz)
    constant_y = 0.26451;
    d12x = 1.3; d12z = 95;
    d23x = -133.3; d23z = 0.5;
    d3eex = -126.994; d3eez = 2.8614;

    S = 126.994^2 + 2.8614^2;
    K0 = 133.3^2 + 0.5^2 + S;
    K1 = 2 * (133.3*126.994 + 0.5*2.8614);
    K2 = 2 * (-133.3*2.8614 + 0.5*126.994);

    r = sqrt(px^2 + py^2);
    solutions = [];

    if abs(constant_y) > r
        return; 
    end

    phi = atan2(px, py);
    gamma = acos(constant_y / r);
    theta1_candidates = [-phi + gamma; -phi - gamma];

    for theta1 = theta1_candidates'
        a = cos(theta1);
        b = sin(theta1);
        W = a*px + b*py;
        M = W - d12x;
        N = pz - d12z;
        R_val = M^2 + N^2;

        RHS = R_val - K0;
        normK = sqrt(K1^2 + K2^2);
        if abs(RHS) > normK
            continue;
        end
        alpha = atan2(K2, K1);
        beta = acos(RHS / normK);
        theta3_candidates = [alpha + beta; alpha - beta];

        for theta3 = theta3_candidates'
            U = cos(theta3);
            V = sin(theta3);
            A = d23x + d3eex*U + d3eez*V;
            B = d23z - d3eex*V + d3eez*U;
            denom = A^2 + B^2;

            if denom < 1e-6
                continue;
            end
            cos_theta2 = (A*M + B*N) / denom;
            sin_theta2 = (B*M - A*N) / denom;
            theta2 = atan2(sin_theta2, cos_theta2);

            solutions = [solutions; theta1, theta2, theta3];
        end
    end
end
