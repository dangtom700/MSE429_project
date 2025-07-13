%% -------------------- DYNAMIC PROPERTIES --------------------
% Link 1 (Base Link)
Link1.mass = 0.21025; % kg
Link1.volume = 0.00021025094; % m^3
Link1.surface_area = 0.12912587; % m^2
Link1.com = [0.00512, 0.00097, 0.05928]; % m

% Principal axes remain the same (unitless rotation matrix)
Link1.principal_axes = [0.15,  0.03,  0.99;
                       -0.99,  0.01,  0.15;
                        0,    -1,     0.03];

% Principal moments of inertia [kg·m^2]
Link1.principal_moments = [123568.96; 377166.30; 390310.51] * 1e-9;

% Inertia tensor at CoM [kg·m^2]
Link1.inertia_com = [371584.58,  1133.32,   37190.55;
                     1133.32,   390041.76,  8369.32;
                     37190.55,   8369.32,   129419.43] * 1e-9;

% Inertia tensor at joint frame [kg·m^2]
Link1.inertia_joint = [1110631.36,  2180.71,    101000.38;
                       2180.71,    1134400.35,  20496.89;
                       101000.38,   20496.89,   135129.36] * 1e-9;

% ------------------------------------------------------------------------
% Link 2 (Middle Link)
Link2.mass = 0.11181; % kg
Link2.volume = 0.00011180654; % m^3
Link2.surface_area = 0.06482837; % m^2
Link2.com = [0.00138, 0.10492, 0.00001]; % m

Link2.principal_axes = [0.15,  -0.99,  0;
                        0.99,  0.15,   0.01;
                       -0.01,  0,      1];

Link2.principal_moments = [49369.53; 234039.74; 250997.05] * 1e-9;

Link2.inertia_com = [230119.67, -26620.50,  85.99;
                    -26620.50,   53290.16,  76.49;
                     85.99,      76.49,     250996.49] * 1e-9;

Link2.inertia_joint = [1460976.05, -10443.01,  88.16;
                      -10443.01,    53502.81,  241.58;
                       88.16,      241.58,    1482065.47] * 1e-9;

% ------------------------------------------------------------------------
% Link 3 (End-Effector Link)
Link3.mass = 0.03816; % kg
Link3.volume = 0.00003815756; % m^3
Link3.surface_area = 0.01950787; % m^2
Link3.com = [0.01352, 0.06237, 0.00001]; % m

Link3.principal_axes = [0.01, -1,     0;
                        0,     0,    -1;
                        1,     0.01,  0];

Link3.principal_moments = [3898.27; 65335.24; 67681.49] * 1e-9;

Link3.inertia_com = [67677.77, -487.23,  1.61;
                    -487.23,   3902,     9.04;
                     1.61,     9.04,     65335.24] * 1e-9;

Link3.inertia_joint = [216094.85, 31676.03,  4.52;
                       31676.03,  10872.06,  22.50;
                       4.52,      22.50,     220722.38] * 1e-9;

% ------------------------------------------------------------------------
% Object
test_tube.mass = 0.00668; % kg
test_tube.volume = 0.00000735866; % m^3
test_tube.surface_area = 0.00751469; % m^2
test_tube.com = [0, 0, -0.04123]; % m

test_tube.principal_axes = [0, 0, 1;
                            -1, 0, 0;
                             0, -1, 0];

test_tube.principal_moments = [326.04, 4057.51, 4057.51] * 1e-9;

test_tube.inertia_com = [4057.51, 0, 0;
                         0, 4057.51, 0;
                         0, 0, 326.04] * 1e-9;

test_tube.inertia_joint = [15414.68, 0, 0;
                           0, 15414.68, 0;
                           0, 0, 326.04] * 1e-9;
