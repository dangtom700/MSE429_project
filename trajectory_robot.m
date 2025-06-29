clc; close all; clear;

%% 1. Robot Configuration
% DH Parameters [Joint1, Joint2, Joint3, Joint4, Tool]
d = [67, 30, 12.7, 35, 0];            % Link offsets (mm)
r = [0, 30, 109, 116, 11.5];          % Link lengths (mm)
alpha = [0, pi/2, 0, -pi/2, 0];       % Link twists (rad)

%% 2. Initialize Figure with Multiple Views
fig = figure('Name', 'Robot Arm Simulation', 'Position', [100 100 1200 800]);

% Create 3D view
ax3D = subplot(2,2,[1,3]);
hold(ax3D, 'on');
grid(ax3D, 'on');
axis(ax3D, 'equal');
xlabel(ax3D, 'X (mm)'); ylabel(ax3D, 'Y (mm)'); zlabel(ax3D, 'Z (mm)');
title(ax3D, '3D View');
view(ax3D, 3);

% Create 2D views
axXY = subplot(2,2,2);
hold(axXY, 'on'); grid(axXY, 'on'); axis(axXY, 'equal');
title(axXY, 'XY Plane'); xlabel('X (mm)'); ylabel('Y (mm)');

axYZ = subplot(2,2,4);  % Changed from axXZ to axYZ
hold(axYZ, 'on'); grid(axYZ, 'on'); axis(axYZ, 'equal');
title(axYZ, 'YZ Plane'); xlabel('Y (mm)'); ylabel('Z (mm)');

% Set consistent axis limits
max_reach = sum(r(2:4)) + d(1);
axis_limits = [-max_reach max_reach -max_reach max_reach -50 max_reach*1.5];
axis(ax3D, axis_limits);
xlim(axXY, axis_limits(1:2));
ylim(axXY, axis_limits(3:4));
xlim(axYZ, axis_limits(3:4));  % Y axis limits for YZ plot
ylim(axYZ, axis_limits(5:6));  % Z axis limits for YZ plot

% Create base visualization in 3D view
[Xb, Yb, Zb] = cylinder(10, 20);
Zb = Zb * d(1);
surf(ax3D, Xb, Yb, Zb, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');

%% 3. Initialize Robot Visualization
% 3D View Components
link_handles_3D = gobjects(3,1);
joint_handles_3D = gobjects(3,1);
for i = 1:3
    link_handles_3D(i) = plot3(ax3D, [0 0], [0 0], [0 0], 'b-', 'LineWidth', 4);
    joint_handles_3D(i) = plot3(ax3D, 0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end
ee_handle_3D = plot3(ax3D, 0, 0, 0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
traj_handle_3D = plot3(ax3D, 0, 0, 0, 'k:', 'LineWidth', 1.5);

% 2D View Components (XY plane)
traj_handle_XY = plot(axXY, 0, 0, 'k:', 'LineWidth', 1.5);
ee_handle_XY = plot(axXY, 0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% 2D View Components (YZ plane)
traj_handle_YZ = plot(axYZ, 0, 0, 'k:', 'LineWidth', 1.5);
ee_handle_YZ = plot(axYZ, 0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

%% 4. Animation Parameters
num_points = 50;
velocity_deg = randi([-90, 90], 1, 3); % Degrees per iteration
current_angles_deg = [0, 30, -30]; % Start with non-zero angles for better visualization

% Storage for trajectories
ee_positions = zeros(num_points, 3);
ee_positions_XY = zeros(num_points, 2);
ee_positions_YZ = zeros(num_points, 2);

%% 5. Main Animation Loop
for i = 1:num_points
    % Update joint angles
    current_angles_deg = current_angles_deg + velocity_deg;
    %current_angles_deg = mod(current_angles_deg, 360);
    current_angles_rad = deg2rad(current_angles_deg);
    
    % Compute forward kinematics
    T = dh_transform([0, current_angles_rad, 0], d, r, alpha);
    
    % Get joint and end effector positions
    P0 = [0; 0; 0; 1];
    P1 = T{1} * [0; 0; 0; 1];
    P2 = T{2} * [0; 0; 0; 1];
    P3 = T{3} * [0; 0; 0; 1];
    
    % Store positions
    ee_positions(i,:) = P3(1:3)';
    ee_positions_XY(i,:) = P3(1:2)';
    ee_positions_YZ(i,:) = P3(2:3)';
    
    %% Update 3D Visualization
    set(link_handles_3D(1), 'XData', [P0(1) P1(1)], 'YData', [P0(2) P1(2)], 'ZData', [P0(3) P1(3)]);
    set(link_handles_3D(2), 'XData', [P1(1) P2(1)], 'YData', [P1(2) P2(2)], 'ZData', [P1(3) P2(3)]);
    set(link_handles_3D(3), 'XData', [P2(1) P3(1)], 'YData', [P2(2) P3(2)], 'ZData', [P2(3) P3(3)]);
    
    set(joint_handles_3D(1), 'XData', P0(1), 'YData', P0(2), 'ZData', P0(3));
    set(joint_handles_3D(2), 'XData', P1(1), 'YData', P1(2), 'ZData', P1(3));
    set(joint_handles_3D(3), 'XData', P2(1), 'YData', P2(2), 'ZData', P2(3));
    
    set(ee_handle_3D, 'XData', P3(1), 'YData', P3(2), 'ZData', P3(3));
    set(traj_handle_3D, 'XData', ee_positions(1:i,1), 'YData', ee_positions(1:i,2), 'ZData', ee_positions(1:i,3));
    
    %% Update 2D Visualizations
    % XY Plane
    set(ee_handle_XY, 'XData', P3(1), 'YData', P3(2));
    set(traj_handle_XY, 'XData', ee_positions_XY(1:i,1), 'YData', ee_positions_XY(1:i,2));
    
    % YZ Plane
    set(ee_handle_YZ, 'XData', P3(2), 'YData', P3(3));  % Plot Y vs Z
    set(traj_handle_YZ, 'XData', ee_positions_YZ(1:i,1), 'YData', ee_positions_YZ(1:i,2));
    
    % Update title with current angles
    view_degree = mod(current_angles_deg, 360);
    title(ax3D, sprintf('3D View\nAngles: [%.1f°, %.1f°, %.1f°]\nVelocity: [%d, %d, %d]\n', ...
        view_degree(1), view_degree(2), view_degree(3), velocity_deg(1),velocity_deg(2),velocity_deg(3)));
    
    drawnow;
    pause(0.02);
end

%% 6. Add XZ Plane Visualization (After Animation)
fig2 = figure('Name', 'XZ Plane Trajectory', 'Position', [100 100 600 600]);
axXZ = axes;
hold(axXZ, 'on'); grid(axXZ, 'on'); axis(axXZ, 'equal');
title(axXZ, 'XZ Plane Trajectory');
xlabel('X (mm)'); ylabel('Z (mm)');

% Plot XZ trajectory
ee_positions_XZ = [ee_positions(:,1), ee_positions(:,3)];
plot(axXZ, ee_positions_XZ(:,1), ee_positions_XZ(:,2), 'k:', 'LineWidth', 1.5);
plot(axXZ, ee_positions_XZ(end,1), ee_positions_XZ(end,2), 'go', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'g');

%% Helper Functions
function T = dh_transform(theta, d, a, alpha)
    % Compute forward kinematics using DH parameters
    n = length(theta);
    T = cell(n,1);
    
    for i = 1:n
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        
        T{i} = [ct, -st*ca, st*sa, a(i)*ct;
                st, ct*ca, -ct*sa, a(i)*st;
                0, sa, ca, d(i);
                0, 0, 0, 1];
        
        if i > 1
            T{i} = T{i-1} * T{i};
        end
    end
end