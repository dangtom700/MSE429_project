close all; clc;

%% Load STL file Link1.STL (Correct)
stlData = stlread('Link1.STL');  % returns a structure with Faces and Vertices
F = stlData.ConnectivityList;
V = stlData.Points;

% Define the reference point
ref_point = [97.9527, 34.4903, 134.099];

% Shift vertices so that reference point moves to origin
V = V - ref_point;

% Save mesh to .mat and reload (optional)
save('Link1.mat', 'F', 'V');
load('Link1.mat');  % loads F and V

% Create an object structure with fields F and V
object.F = F;
object.V = V;

% Store and retrieve using app data (optional)
setappdata(0, 'object_data', object);
object = getappdata(0, 'object_data');

% Store in cell array for multi-part manipulators
obj = cell(1, 5);     % Assuming up to 5 parts
obj{3} = object;      % Assign Link1 as part 3

% Create figure
fig_handle = figure;

% Plot part 3
q(3) = patch('Faces', obj{3}.F, 'Vertices', obj{3}.V, ...
             'FaceColor', [0.6, 0.6, 0.4], 'EdgeColor', 'none');

% Store transposed vertices in cell array
V_cell = cell(1, 5);
V3 = obj{3}.V';
V_cell{3} = V3;

% Visualization settings
set(fig_handle, 'Renderer', 'zbuffer', 'DoubleBuffer', 'off');
light('Color', [0.99, 0.99, 0.99], 'Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;
daspect([1 1 1]);
view(3);
rotate3d on;
axis([-100 100 -100 100 0 150]);  % Fixed 3D axis limits

% Define point and transform to local coordinates
pt_global = [1.3, 40, 95];       % original point in global coords
pt_local = pt_global;  % shift to local coordinates

% Create a sphere at that point
[r_sphere, v_sphere] = meshgrid(linspace(0, pi, 15), linspace(0, 2*pi, 30));
x_s = 10 * sin(r_sphere) .* cos(v_sphere) + pt_local(1);
y_s = 10 * sin(r_sphere) .* sin(v_sphere) + pt_local(2);
z_s = 10 * cos(r_sphere) + pt_local(3);

% Plot the sphere and store handle
hold on;
h_sphere = surf(x_s, y_s, z_s, 'FaceColor', 'r', 'EdgeColor', 'none');

% Rotation animation about Z-axis
theta = linspace(0, 2*pi, 180);  % 180 frames

for k = 1:length(theta)
    % Rotation matrix around Z-axis
    Rz = [cos(theta(k)), -sin(theta(k)), 0;
          sin(theta(k)),  cos(theta(k)), 0;
                     0,             0, 1];

    % Rotate STL component
    V_rotated = Rz * V_cell{3};
    set(q(3), 'Vertices', V_rotated');

    % Rotate the sphere center
    pt_rotated = (Rz * pt_local')';

    % Recompute the rotated sphere vertices
    x_s = 25 * sin(r_sphere) .* cos(v_sphere) + pt_rotated(1);
    y_s = 25 * sin(r_sphere) .* sin(v_sphere) + pt_rotated(2);
    z_s = 25 * cos(r_sphere) + pt_rotated(3);

    % Update the sphere surface
    set(h_sphere, 'XData', x_s, 'YData', y_s, 'ZData', z_s);

    drawnow;
    pause(0.01);
end

close all;
clc;

%% Load STL file Link2.STL
stlData = stlread('Link2.STL');  % Structure with Faces and Points
F = stlData.ConnectivityList;
V = stlData.Points;

% Reference setup
ref_point = [157.325, 42.4651, 173.659];    % Local reference point to zero
target_position = [1.3, 40, 95];            % Final global position

% Step 1: Move reference to origin
V = V - ref_point;

% Step 2: Rotate 90 deg around X-axis at origin
Rx = [1, 0, 0;
      0, 0, -1;
      0, 1, 0];
V = (Rx * V')';  % Rotate and transpose back

% Store original for animation
V_original = V';

% Plot setup
fig_handle = figure;
q = patch('Faces', F, 'Vertices', V, ...
          'FaceColor', [0.4, 0.6, 0.8], 'EdgeColor', 'none');
      
hold on;
light('Position', [0 0 2], 'Style', 'infinite');
lighting gouraud;
axis vis3d;
axis equal;
axis([-250 250 -250 250 -250 250])
view(3);
rotate3d on;

% Animation: Rotate about Z axis at origin
theta = linspace(0, 2*pi, 180);

for k = 1:length(theta)
    % Rotation about Z axis
    Rz = [cos(theta(k)), 0, sin(theta(k));
          0, 1, 0;
          -sin(theta(k)), 0, cos(theta(k))];

    % Apply rotation
    V_rot = Rz * V_original + target_position';

    % Update the patch
    set(q, 'Vertices', V_rot');
    
    drawnow;
    pause(0.01);
end

%% Load STL file Link3.STL
stlData = stlread('Link3.STL');  % Structure with Faces and Points
F = stlData.ConnectivityList;
V = stlData.Points;

% Reference setup
ref_point = [127.04, 113.902, 187.562];    % Local reference point to zero
target_position = [-132, 12.5,95.5];       % Final global position

% Step 1: Move reference to origin
V = V - ref_point;

% Step 2: Rotate 90 deg around X-axis at origin
Rx = [1, 0, 0;
      0, 0, -1;
      0, 1, 0];
V = (Rx * V')';  % Rotate and transpose back

% Store original for animation
V_original = V';

% Plot setup
fig_handle = figure;
q = patch('Faces', F, 'Vertices', V, ...
          'FaceColor', [0.4, 0.6, 0.8], 'EdgeColor', 'none');
      
hold on;
light('Position', [0 0 2], 'Style', 'infinite');
lighting gouraud;
axis vis3d;
axis equal;
axis([-250 250 -250 250 -250 250])
view(3);
rotate3d on;

% Animation: Rotate about Z axis at origin
theta = linspace(0, 2*pi, 180);

for k = 1:length(theta)
    % Rotation about Z axis
    Rz = [cos(theta(k)), 0, sin(theta(k));
          0, 1, 0;
          -sin(theta(k)), 0, cos(theta(k))];

    % Apply rotation
    V_rot = Rz * V_original + target_position';

    % Update the patch
    set(q, 'Vertices', V_rot');
    
    drawnow;
    pause(0.01);
end
