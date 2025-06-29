close all; clc;

%% ----- SETUP FIGURE -----
fig = figure('Renderer', 'zbuffer');
axis equal; axis vis3d; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); rotate3d on;
axis([-300 300 -300 300 -200 350]); % Adjusted for better visibility

% Lighting
light('Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;

%% ----- COLORS -----
link_colors = {[0.6 0.6 0.4], [0.4 0.6 0.8], [0.8 0.4 0.6]};
frame_colors = {'r', 'g', 'b'};

%% ----- LOAD STL LINKS -----
% Define joint home positions (base frame)
joint1_home = [0, 0, 0];
joint2_home = [1.3, 40, 95];
joint3_home = [-132, 12.5, 95.5];

% Compute relative translations between joints
d12 = joint2_home - joint1_home;   % [1.3, 40, 95]
d23 = joint3_home - joint2_home;   % [-133.3, -27.5, 0.5]

% Load links WITHOUT target_position
[L1.F, L1.V] = load_link('Link1.STL', [97.9527, 34.4903, 134.099], eye(3));
[L2.F, L2.V] = load_link('Link2.STL', [157.325, 42.4651, 173.659], [1 0 0; 0 0 -1; 0 1 0]);
[L3.F, L3.V] = load_link('Link3.STL', [127.04, 113.902, 187.562], [1 0 0; 0 0 -1; 0 1 0]);

% Transpose vertices for animation
L1.V0 = L1.V';
L2.V0 = L2.V';
L3.V0 = L3.V';

% Create patch objects
q(1) = patch('Faces', L1.F, 'Vertices', L1.V0', ...
             'FaceColor', link_colors{1}, 'EdgeColor', 'none');
q(2) = patch('Faces', L2.F, 'Vertices', L2.V0', ...
             'FaceColor', link_colors{2}, 'EdgeColor', 'none');
q(3) = patch('Faces', L3.F, 'Vertices', L3.V0', ...
             'FaceColor', link_colors{3}, 'EdgeColor', 'none');

%% ----- ANIMATION LOOP -----
trace_pts = struct('L1', [], 'L2', [], 'L3', []);
iteration = 100;
theta = [deg2rad(linspace(0,270,iteration));
         deg2rad(linspace(0,135, iteration));
         deg2rad(linspace(-45,135, iteration))];

for k = 1:iteration
    % Joint 1: Base rotation (Z-axis)
    R1 = rotz(rad2deg(theta(1,k)));
    T1 = [R1, [0;0;0]; 0 0 0 1]; % No translation for base
    
    % Joint 2: Rotation about Y (relative to Link1)
    R2 = roty(rad2deg(theta(2,k)));
    T2 = [R2, d12'; 0 0 0 1]; % Relative translation d12
    
    % Joint 3: Rotation about Y (relative to Link2)
    R3 = roty(rad2deg(theta(3,k)));
    T3 = [R3, d23'; 0 0 0 1]; % Relative translation d23
    
    % Apply transforms
    V1_t = apply_transform(L1.V0, T1);
    V2_t = apply_transform(L2.V0, T1 * T2);
    V3_t = apply_transform(L3.V0, T1 * T2 * T3);
    
    % Update geometry
    set(q(1), 'Vertices', V1_t');
    set(q(2), 'Vertices', V2_t');
    set(q(3), 'Vertices', V3_t');
    
    % Store joint positions (corrected)
    joint1 = T1(1:3,4)';
    joint2 = (T1 * T2);
    joint2 = joint2(1:3,4)';
    joint3 = (T1 * T2 * T3);
    joint3 = joint3(1:3,4)';
    
    trace_pts.L1 = [trace_pts.L1; joint1];
    trace_pts.L2 = [trace_pts.L2; joint2];
    trace_pts.L3 = [trace_pts.L3; joint3];
    
    % Plot trajectories
    plot3(trace_pts.L1(:,1), trace_pts.L1(:,2), trace_pts.L1(:,3), 'r.', 'MarkerSize', 1);
    plot3(trace_pts.L2(:,1), trace_pts.L2(:,2), trace_pts.L2(:,3), 'g.', 'MarkerSize', 1);
    plot3(trace_pts.L3(:,1), trace_pts.L3(:,2), trace_pts.L3(:,3), 'b.', 'MarkerSize', 1);
    
    % Draw coordinate frames
    draw_frame(T1, 30);
    draw_frame(T1*T2, 30);
    draw_frame(T1*T2*T3, 30);
    
    drawnow;
end

%% ---- HELPER FUNCTIONS ----
function [F, V] = load_link(filename, ref_point, R_align)
    data = stlread(filename);
    V = data.Points - ref_point;      % Center at joint origin
    V = (R_align * V')';              % Apply alignment rotation
    F = data.ConnectivityList;
    % NOTE: target_position NOT added here!
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
    x_axis = T(1:3,1) * L;
    y_axis = T(1:3,2) * L;
    z_axis = T(1:3,3) * L;
    line([origin(1) origin(1)+x_axis(1)], [origin(2) origin(2)+x_axis(2)], [origin(3) origin(3)+x_axis(3)], 'Color', 'r', 'LineWidth', 2);
    line([origin(1) origin(1)+y_axis(1)], [origin(2) origin(2)+y_axis(2)], [origin(3) origin(3)+y_axis(3)], 'Color', 'g', 'LineWidth', 2);
    line([origin(1) origin(1)+z_axis(1)], [origin(2) origin(2)+z_axis(2)], [origin(3) origin(3)+z_axis(3)], 'Color', 'b', 'LineWidth', 2);
end
