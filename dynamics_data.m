%{
Dynamics properties of this robot arm
- Material: PE (only applied for the costumed-made components - links) 
- Purpose: resilient material against chemical and toxic environment, 
reasonable price per kg, light weight system design, acceptable mechanical
properties, such as Young's module, maximum tensile strength, fatigue
toughness, fracture strength.
- Unit: measurements conducted is in millimeters for length and grams for
mass. Mass (grams), volume (cubic millimeters), surface area (square
millimeters), center of mass (millimeters), moment of inertia (grams *
square millimeters)
- Abbreviation: com (center of mass), PAI (principal axes of inertia), PMI
(principal moments of inertia), MICOM (moments of inertia at center of
mass), MICS (moments of inertia at output coordinate system)
%}

%% ------------ WHOLE SYSTEM ------------
whole_sys.mass = 349.97;
whole_sys.volume = 360215.03;
whole_sys.surface_area = 213462.11;
whole_sys.PAI = [-0.10, -0.96, -0.26; % Ix
                 -0.37, 0.27, -0.89;  % Iy
                 0.92, 0.01, -0.39];  % Iz
whole_sys.PMI = [529673.31, 2143094.30, 2357570.25];

% Coordinate 1
whole_sys.com_1 = [15.13, 53.88, 75.09];
whole_sys.MICOM_1 = [2310698.96, 146823.19, 116301.73;
                     146823.19, 651705.71, 400612.34;
                     116301.73, 400612.34, 2067933.18];
whole_sys.MICS_1 = [5299980.18, 432153.32, 513953.57;
                    432153.32, 2705143.91, 1816532.83;
                    513953.57, 1816532.83, 3164042.34];

% Coordinate 2
whole_sys.com_2 = [-35.40, 53.86, -22.21];
whole_sys.MICOM_2 = [2310857.77, 145623.11, 116084.38;
                     145923.11, 651546.90, 400675.38;
                     116084.38, 400675.38, 2067933.18];
whole_sys.MICS_2 = [3498787.26, -521351.91, 116084.38;
                    -521351.91, 1262790.94, -18026.11;
                    391276.13, -18026.11, 3521752.27];

% Coordinate 3
whole_sys.com_3 = [16.17, -75.81, -22.21];
whole_sys.MICOM_3 = [2310857.77, 145923.11, 116084.38;
                     145923.11, 651546.90, 400675.38;
                     116084.38, 400675.38, 400675.38;
                     116084.38, 400675.38, 2067933.18];
whole_sys.MICS_3 = [4494802.86, -283095.78, -9622.48;
                    -283095.78, 915737.14, 989997.14;
                    -9622.48, 989997.14, 4170714.08];

%% ------------ LINK 1 (BASE) ------------
whole_sys.mass = 204.72;
whole_sys.volume = 210250.93;
whole_sys.surface_area = 129125.87;
whole_sys.PAI = [0.16, -0.01, 0.99; % Ix
                 -0.99, 0.01, 0.16;  % Iy
                 -0.01, -1.00, -0.01];  % Iz
whole_sys.PMI = [119720.15, 364493.75, 383151.27];

% Coordinate 1
whole_sys.com_1 = [4.90, 1.77, 59.45];
whole_sys.MICOM_1 = [358222.61, -396.84, 38675.05;
                     -396.84, 383138.34, -1757369;
                     38675.05, -1757.69, 126004.21];
whole_sys.MICS_1 = [1082296.13, 1379.86, 98267.34;
                    1379.86, 1111477.70, 19810.83;
                    98267.34, 19810.83, 126004.15];

% Coordinate 2
whole_sys.com_2 = [-45.61, 1.75, -37.86];
whole_sys.MICOM_2 = [358222.19, -383.32, 38676.00;
                     -383.32, 383138.76, -1736.71;
                     38676.00, -1736.71, 126004.21];
whole_sys.MICS_2 = [652250.05, -16699.79, 392137.10;
                    -16699.79, 1102354.73, -15280.75;
                    392137.10, -15280.75, 552442.76];

% Coordinate 3
whole_sys.com_3 = [5.96, -127.92, -37.86];
whole_sys.MICOM_3 = [358222.19, -383.32, 38676.00;
                     -383.32, 383138.76, -1736.71;
                     38676.00, -1736.71, 126004.21];
whole_sys.MICS_3 = [4001678.94, -156566.32, -7545.06;
                    -156566.32, 683822.82, 989683.85;
                    -7545.06, 989683.85, 3483339.74];

%% ------------ LINK 2 (MID LINK) ------------
whole_sys.mass = 108.74;
whole_sys.volume = 111806.54;
whole_sys.surface_area = 64828.37;
whole_sys.PAI = [0.15, -0.99, 0.00; % Ix
                 0.99, 0.15, -0.07;  % Iy
                 0.07, 0.01, 1.00];  % Iz
whole_sys.PMI = [48797.36, 224964.61, 242225.00];

% Coordinate 1
whole_sys.com_1 = [39.15, 105.49, 97.09];
whole_sys.MICOM_1 = [221203.36, -25729.69, -1140.25;
                     -25729.69, 52637.13, -228.09;
                     -1140.25, -228.09, 242146.48];
whole_sys.MICS_1 = [2456327.37, 423355.05, 412169.58;
                    423355.05, 1244297.47, 1113494.27;
                    412169.58, 1113494.27, 1618926.56];

% Coordinate 2
whole_sys.com_2 = [-11.41, 105.48, -0.21];
whole_sys.MICOM_2 = [221175.40, -25820.72, -1140.13;
                     -25820.72, 52665.09, -228.71;
                     -1140.13, -228.71, 242146.48];
whole_sys.MICS_2 = [1431160.62, -156714.28, -873.47;
                    -156714.28, 66829.95, -2693.72;
                    -873.47, -2693.72, 242146.48];

% Coordinate 3
whole_sys.com_3 = [40.16, -24.18, -0.21];
whole_sys.MICOM_3 = [221175.40, -25820.72, -1140.13;
                     -25820.72, 52665.09, -228.71;
                     -1140.13, -228.71, 242146.48];
whole_sys.MICS_3 = [284781.21, -131433.29, -2078.58;
                    -131433.29, 228045.52, 336.44;
                    -2078.58, 336.44, 481122.67];

%% ------------ LINK 3 (END EFFECTOR) ------------
whole_sys.mass = 36.51;
whole_sys.volume = 38157.56;
whole_sys.surface_area = 19507.87;
whole_sys.PAI = [-0.01, -1.00, 0.00; % Ix
                 0.00, 0.00, -1.00;  % Iy
                 1.00, -0.01, 0.00];  % Iz
whole_sys.PMI = [3726.42, 62803.60, 65037.03];

% Coordinate 1
whole_sys.com_1 = [0.99, 192.35, 97.30];
whole_sys.MICOM_1 = [65033.44, 469.32, 1.59;
                     469.32, 3730.02, -9.38;
                     1.59, -9.38, 62803.60];
whole_sys.MICS_1 = [1761356.68, 7418.41, 3516.66;
                    7418.41, 349368.75, 683227.73;
                    3516.66, -9.38, 62803.60];

% Coordinate 2
whole_sys.com_2 = [-49.62, 192.32, -0.01];
whole_sys.MICOM_2 = [65033.93, 436.06, 1.59;
                     436.06, 3729.52, -9.38;
                     1.59, -9.38, 62803.60];
whole_sys.MICS_2 = [1415376.59, -347937.85, 12.50;
                    -347937.85, 93606.25, -51.64;
                    12.50, -51.64, 1503022.99];

% Coordinate 3
whole_sys.com_3 = [1.95, 62.65, -0.01];
whole_sys.MICOM_3 = [65033.93, 436.06, 1.59;
                     436.06, 3729.52, -9.38;
                     1.59, -9.38, 62803.60];
whole_sys.MICS_3 = [208342.71, 4903.82, 1.16;
                    4903.82, 3868.81, -23.14;
                    1.16, -23.14, 206251.67];