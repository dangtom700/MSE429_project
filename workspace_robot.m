clc; 
close all; 
clear;

%% Configuration Parameters
% DH Parameters [Joint1, Joint2, Joint3, Joint4, Tool]
d = [95, 0.5, 0];            % Link offsets (mm)
r = [40, 140, 125];          % Link lengths (mm)
alpha = [pi/2, 0, -pi/2];       % Link twists (rad)

%% Test Case Validation
% Test with known angles (30°, 45°, 60°)
test_angles_deg = [30, 45, 60];
test_angles_rad = deg2rad(test_angles_deg);

% Compute transforms using both methods
T_dh = dh_transform(test_angles_rad, d, r, alpha);
T_basetool = BaseToTool(test_angles_deg(1), test_angles_deg(2), test_angles_deg(3));

% Display validation results
disp('=== Validation Test ===');
disp('DH Transform:'); disp(T_dh);
disp('BaseToTool:'); disp(T_basetool);
disp('Difference:'); disp(T_dh - T_basetool);
disp(['Sum of absolute differences: ', num2str(sum(sum(abs(T_dh - T_basetool))))]);

%% Trajectory Comparison
% Simulation parameters
velocity_deg = randi([-180, 180], 1, 3);% Degrees per second
time_step = 0.001;                      % Time step (s)
iterations = 100;                       % Number of iterations

% Initialize storage
location_dh = [];
location_bt = [];
differences = zeros(iterations, 1);

% Initial joint angles (degrees)
current_angles_deg = [0, 0, 0];

% Start simulation
fprintf("Running simulation at angular velocity (degree): %.4f, %.4f, %.4f\n", velocity_deg(1), velocity_deg(2), velocity_deg(3))
for i = 1:iterations
    % Update angles
    current_angles_deg = current_angles_deg + time_step * velocity_deg;
    current_angles_rad = deg2rad(current_angles_deg);
    
    % Compute transforms
    T_dh_current = dh_transform(current_angles_rad, d, r, alpha);
    T_bt_current = BaseToTool(current_angles_deg(1), current_angles_deg(2), current_angles_deg(3));
    
    % Store positions
    location_dh = [location_dh; T_dh_current(1:3, 4)'];
    location_bt = [location_bt; T_bt_current(1:3, 4)'];
    
    % Calculate difference
    differences(i) = norm(T_dh_current(1:3, 4) - T_bt_current(1:3, 4));
    
    % Display progress
    if mod(i, 10) == 0
        fprintf('Iteration %d: Current angles [%.1f, %.1f, %.1f]°, Difference: %.4f mm\n', ...
                i, current_angles_deg(1), current_angles_deg(2), current_angles_deg(3), differences(i));
    end
end

%% Plotting Results
% Difference plot
figure;
plot(differences, 'LineWidth', 2);
title('Euclidean Distance Between DH and BaseToTool');
xlabel('Iteration'); ylabel('Position Difference (mm)');
grid on; ylim([0, max(differences)*1.1]);

% Trajectory comparison
figure;
subplot(1,2,1);
scatter3(location_dh(:,1), location_dh(:,2), location_dh(:,3), 10, 'filled');
title('DH Transform Trajectory');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; axis equal;

subplot(1,2,2);
scatter3(location_bt(:,1), location_bt(:,2), location_bt(:,3), 10, 'filled');
title('BaseToTool Trajectory');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; axis equal;

%% Workspace Analysis
% Define angle ranges (degrees)
angle_ranges = struct(...
    'theta1', 0:5:270, ...    % Joint 1 range
    'theta2', 0:5:135, ...    % Joint 2 range
    'theta3', -135:5:45);     % Joint 3 range

% Generate workspace
workspace_points = generateWorkspace(angle_ranges);

% Plot workspace
plotWorkspace(workspace_points);

%% Workspace Range Analysis (Spherical Coordinates)

% Extract workspace positions
X = workspace_points(:,1);
Y = workspace_points(:,2);
Z = workspace_points(:,3);

% Convert to spherical coordinates
radius = sqrt(X.^2 + Y.^2 + Z.^2);            % r
azimuth = atan2d(Y, X);                       % θ: angle in X-Y plane
elevation = atan2d(Z, sqrt(X.^2 + Y.^2));     % φ: angle from X-Y plane

% Calculate min and max for each
radius_range = [min(radius), max(radius)];
azimuth_range = [min(azimuth), max(azimuth)];
elevation_range = [min(elevation), max(elevation)];

% Display results
fprintf('\n=== Workspace Spherical Range ===\n');
fprintf('Radius (r):        Min = %.2f mm, Max = %.2f mm\n', radius_range(1), radius_range(2));
fprintf('Azimuth (θ):       Min = %.2f°, Max = %.2f°\n', azimuth_range(1), azimuth_range(2));
fprintf('Elevation (φ):     Min = %.2f°, Max = %.2f°\n', elevation_range(1), elevation_range(2));
fprintf("X-range:           Min = %.2f, Max = %.2f\n", min(X), max(X));
fprintf("Y-range:           Min = %.2f, Max = %.2f\n", min(Y), max(Y));
fprintf("Z-range:           Min = %.2f, Max = %.2f\n", min(Z), max(Z));

%% Helper Functions
function T = dh_transform(theta, d, a, alpha)
    % Standard DH transformation
    % Inputs in radians
    n = length(theta);
    T = eye(4);
    
    for i = 1:n
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        
        Ti = [ct, -st*ca,  st*sa, a(i)*ct;
              st,  ct*ca, -ct*sa, a(i)*st;
               0,     sa,     ca,     d(i);
               0,      0,      0,       1];
           
        T = T * Ti;
    end
end

function T = BaseToTool(theta1_deg, theta2_deg, theta3_deg)
    % Closed-form forward kinematics solution
    % Inputs in degrees (converted internally)
    
    % Convert to radians
    theta1 = deg2rad(theta1_deg);
    theta2 = deg2rad(theta2_deg);
    theta3 = deg2rad(theta3_deg);
    
    % Shorthand notation
    c1 = cos(theta1); s1 = sin(theta1);
    c2 = cos(theta2); s2 = sin(theta2);
    c3 = cos(theta3); s3 = sin(theta3);
    
    % Combined terms
    c23 = c2*c3 - s2*s3;
    s23 = s2*c3 + c2*s3;
    
    % Position components
    px = 40*c1 + 140*c1*c2 + 125*c1*c23 + 0.5*s1;
    py = 40*s1 + 140*s1*c2 + 125*s1*c23 - 0.5*c1;
    pz = 125*s23 + 140*s2 + 95;
    
    % Rotation matrix
    T = [c1*c23, -s1, -c1*s23, px;
         s1*c23, c1, -s1*s23, py;
            s23,     0,      c23, pz;
              0,      0,        0,  1];
end

function workspace = generateWorkspace(angles)
    % Generate workspace points
    workspace = [];
    
    for t1 = angles.theta1
        for t2 = angles.theta2
            for t3 = angles.theta3
                T = BaseToTool(t1, t2, t3);
                workspace(end+1, :) = T(1:3, 4)';
            end
        end
    end
end

function plotWorkspace(points)
    % 3D Workspace
    figure;
    scatter3(points(:,1), points(:,2), points(:,3), 3, 'filled');
    title('Robot Workspace');
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    grid on; axis equal; view(3);
    
    % 2D Projections
    figure;
    subplot(2,2,1);
    scatter(points(:,1), points(:,2), 2, 'filled');
    title('X-Y Plane'); xlabel('X'); ylabel('Y');
    axis equal; grid on;
    
    subplot(2,2,2);
    scatter(points(:,1), points(:,3), 2, 'filled');
    title('X-Z Plane'); xlabel('X'); ylabel('Z');
    axis equal; grid on;
    
    subplot(2,2,3);
    scatter(points(:,2), points(:,3), 2, 'filled');
    title('Y-Z Plane'); xlabel('Y'); ylabel('Z');
    axis equal; grid on;
end