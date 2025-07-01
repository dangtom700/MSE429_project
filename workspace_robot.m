clc; 
close all; 
clear;

%% Configuration Parameters
% DH Parameters [Joint1, Joint2, Joint3, Joint4, Tool]
d = [80, 0, 0, 0];            % Link offsets (mm)
r = [0, 6.78, 133.794, 112.268];          % Link lengths (mm)
alpha = [0, pi/2, 0, 0];       % Link twists (rad)

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
location_dh = zeros(iterations, 3);
location_bt = zeros(iterations, 3);
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
    location_dh(i,:) = T_dh_current(1:3,4)';
    location_bt(i,:) = T_bt_current(1:3,4)';
    
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
t1_range = -100:5:100;    % Shoulder rotation
t2_range = -90:5:90;    % Upper arm flexion
t3_range = -90:5:90;  % Forearm flexion
num_samples = numel(t1_range)*numel(t2_range)*numel(t3_range);

% Generate workspace
workspace_points = generateWorkspace(t1_range,t2_range, t3_range, num_samples);

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

function workspace = generateWorkspace(angle1, angle2, angle3, num_samples)
    % Generate workspace points
    workspace = zeros(num_samples, 3);
    index = 0;
    for t1 = angle1
        for t2 = angle2
            for t3 = angle3
                index = index + 1;
                T = BaseToTool(t1, t2, t3);
                workspace(index, :) = T(1:3, 4)';
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