close all;

%% Load STL file Link1.STL
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
axis on;
view(3);
rotate3d on;

%% Load STL file Link2.STL
stlData = stlread('Link2.STL');  % returns a structure with Faces and Vertices
F = stlData.ConnectivityList;
V = stlData.Points;

% Reference point and desired final position
ref_point = [59.3723, 7.97483, 39.5605];
target_position = [0.540418, 48.9454, 77.7329];

% Compute translation vector
translation_vector = target_position - ref_point;

% Apply translation
V = V - ref_point;  % move reference point to origin
V = V + translation_vector;  % move origin to target position

% Rotate 90 degrees around X-axis
Rx = [1, 0, 0;
      0, 0, -1;
      0, 1, 0];
V = (Rx * V')';  % Apply rotation and transpose back

% Save mesh to .mat and reload (optional)
save('Link2.mat', 'F', 'V');
load('Link2.mat');  % loads F and V

% Create an object structure with fields F and V
object.F = F;
object.V = V;

% Store and retrieve using app data (optional)
setappdata(0, 'object_data', object);
object = getappdata(0, 'object_data');

% Store in cell array for multi-part manipulators
obj = cell(1, 5);     % Assuming up to 5 parts
obj{2} = object;      % Assign Link2 as part 2

% Create figure
fig_handle = figure;

% Plot part 2
q(2) = patch('Faces', obj{2}.F, 'Vertices', obj{2}.V, ...
             'FaceColor', [0.4, 0.6, 0.8], 'EdgeColor', 'none');  % Different color for visibility

% Store transposed vertices in cell array
V_cell = cell(1, 5);
V2 = obj{2}.V';
V_cell{2} = V2;

% Visualization settings
set(fig_handle, 'Renderer', 'zbuffer', 'DoubleBuffer', 'off');
light('Color', [0.99, 0.99, 0.99], 'Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;
daspect([1 1 1]);
axis on;
view(3);
rotate3d on;

%% Load STL file Link3.STL
stlData = stlread('Link3.STL');  % returns a structure with Faces and Vertices
F = stlData.ConnectivityList;
V = stlData.Points;

% Reference point and desired final position
ref_point = [127.04, 113.902, 187.562];
target_position = [-92.7181, -212.271, 73.1577];

% Compute translation vector
translation_vector = target_position - ref_point;

% Apply translation
V = V - ref_point;  % move reference point to origin
V = V + translation_vector;  % move origin to target position

% Rotate 90 degrees around X-axis
Rx = [1, 0, 0;
      0, 0, -1;
      0, 1, 0];
V = (Rx * V')';  % Apply rotation and transpose back

% Save mesh to .mat and reload (optional)
save('Link2.mat', 'F', 'V');
load('Link2.mat');  % loads F and V

% Create an object structure with fields F and V
object.F = F;
object.V = V;

% Store and retrieve using app data (optional)
setappdata(0, 'object_data', object);
object = getappdata(0, 'object_data');

% Store in cell array for multi-part manipulators
obj = cell(1, 5);     % Assuming up to 5 parts
obj{2} = object;      % Assign Link2 as part 2

% Create figure
fig_handle = figure;

% Plot part 2
q(2) = patch('Faces', obj{2}.F, 'Vertices', obj{2}.V, ...
             'FaceColor', [0.4, 0.6, 0.8], 'EdgeColor', 'none');  % Different color for visibility

% Store transposed vertices in cell array
V_cell = cell(1, 5);
V2 = obj{2}.V';
V_cell{2} = V2;

% Visualization settings
set(fig_handle, 'Renderer', 'zbuffer', 'DoubleBuffer', 'off');
light('Color', [0.99, 0.99, 0.99], 'Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;
daspect([1 1 1]);
axis on;
view(3);
rotate3d on;
