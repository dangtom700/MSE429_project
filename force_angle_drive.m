close all; clc; clear all;

%% -------------------- DYNAMIC PROPERTIES (SI UNITS) --------------------
g = 9.81; % m/s^2

% Link 1 (Base Link)
Link1.mass = 0.21025; % kg
Link1.com = [0.00512, 0.00097, 0.05928]; % m
Link1.inertia_com = [371584.58,  1133.32,   37190.55;
                     1133.32,   390041.76,  8369.32;
                     37190.55,   8369.32,   129419.43] * 1e-9; % kg·m^2

% Link 2 (Middle Link)
Link2.mass = 0.11181; % kg
Link2.com = [0.00138, 0.10492, 0.00001]; % m
Link2.inertia_com = [230119.67, -26620.50,  85.99;
                    -26620.50,   53290.16,  76.49;
                     85.99,      76.49,     250996.49] * 1e-9; % kg·m^2

% Link 3 (End-Effector Link)
Link3.mass = 0.03816; % kg
Link3.com = [0.01352, 0.06237, 0.00001]; % m
Link3.inertia_com = [67677.77, -487.23,  1.61;
                    -487.23,   3902,     9.04;
                     1.61,     9.04,     65335.24] * 1e-9; % kg·m^2

% Test tube properties
test_tube.mass = 0.00668; % kg
test_tube.com = [0, 0, -0.04123]; % m

% Store all links
robot.links = {Link1, Link2, Link3};
robot.num_links = 3;

%% -------------------- SETUP FIGURE --------------------
figure('Renderer', 'zbuffer');
axis equal; axis vis3d; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); rotate3d on;
axis([-300 300 -300 300 -200 350]);
title("Force-Controlled Path Planning with Torque Analysis")

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

