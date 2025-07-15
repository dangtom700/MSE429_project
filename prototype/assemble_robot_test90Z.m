%{
Define the initial posture of the system for easier dynamics data
integration for force analysis.
Procedure
1) link 1 rotates around z axis
2) link 2 rotates around y axis
3) link 3 rotates around y axis
4) the system is rotate 90 degree around z axis
%}

close all; clear all; clc;

%% Initialize figure and settings
fig_handle = figure('Renderer', 'zbuffer', 'DoubleBuffer', 'off');
light('Color', [0.99, 0.99, 0.99], 'Position', [0, 0, 2], 'Style', 'infinite');
lighting gouraud;
daspect([1 1 1]);
axis equal;
view(3);
rotate3d on;
hold on;

%% Define common parameters
link_colors = {[0.6 0.6 0.4], [0.4 0.6 0.8], [0.8 0.4 0.6]}; % Colors for each link

%% Load Link 1 (Base)
[obj1, V1] = loadSTLComponent('Link1.STL', ...
                             [97.9527, 34.4903, 134.099], ... % Ref point
                             [0,0,0], ...                   
                             [0,1,0; -1,0,0; 0,0,1], ...                       
                             link_colors{1}, 1);

%% Load Link 2 (Arm)
[obj2, V2] = loadSTLComponent('Link2.STL', ...
                             [157.325, 42.4651, 173.659], ... % Ref point
                             [40,-1.3,95], ... 
                             [0,0,-1; -1,0,0; 0,1,0], ...      
                             link_colors{2}, 2);

%% Load Link 3 (Forearm)
[obj3, V3] = loadSTLComponent('Link3.STL', ...
                             [127.04, 113.902, 187.562], ... % Ref point
                             [12.5, 132, 95.5], ... 
                             [0,0,-1; -1,0,0; 0,1,0], ...      
                             link_colors{3}, 3);

%% Verify positions
disp('Link 1 reference point should be at origin:');
disp(mean(obj1.V));

disp('Link 2 reference point should be at target position:');
disp(mean(obj2.V));

disp('Link 3 reference point should be at target position:');
disp(mean(obj3.V));

%% Add coordinate axes for reference
plot3([0 50], [0 0], [0 0], 'r-', 'LineWidth', 2); % X-axis
plot3([0 0], [0 50], [0 0], 'g-', 'LineWidth', 2); % Y-axis
plot3([0 0], [0 0], [0 50], 'b-', 'LineWidth', 2); % Z-axis
text(50,0,0,'X'); text(0,50,0,'Y'); text(0,0,50,'Z');

%% Final visualization settings
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Robotic Manipulator STL Components');
grid on;

function [obj, V_cell] = loadSTLComponent(filename, ref_point, target_position, rotation_matrix, color, link_number)
    % Load STL file
    stlData = stlread(filename);
    
    % Process vertices
    V = stlData.Points - ref_point; % Move to origin
    V = (rotation_matrix * V')';    % Apply rotation
    V = V + target_position;        % Move to target position
    
    % Create object structure
    obj.F = stlData.ConnectivityList;
    obj.V = V;
    
    % Store transposed vertices
    V_cell = V';
    
    % Plot the component
    patch('Faces', obj.F, 'Vertices', obj.V, ...
          'FaceColor', color, 'EdgeColor', 'none');
    
    % Optional: Save to .mat file
    save(sprintf('Link%d.mat', link_number), 'obj');
end
