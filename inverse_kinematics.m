clc;
clear;
close all;

% Test case for validation
test_theta = [30, 30, 40];
target = BaseToTool(test_theta(1), test_theta(2), test_theta(3));
px = target(1,4);
py = target(2,4);
pz = target(3,4);
solutions = inverseKinematics(px, py, pz);
disp("Inverse kinematics solutions (validation):")
disp(solutions)
disp("-----")

if ~isempty(solutions)
    % Initialize best solution tracking
    min_error = Inf;
    best_solution = [];
    best_position_error = Inf;
    best_angle_error = Inf;
    % Evaluate each solution
    for i = 1:size(solutions,1)
        % Convert to degrees for comparison
        sol_deg = rad2deg(solutions(i,:));
        
        % Compute forward kinematics for this solution
        T_solution = BaseToTool(sol_deg(1), sol_deg(2), sol_deg(3));
        px_sol = T_solution(1,4);
        py_sol = T_solution(2,4);
        pz_sol = T_solution(3,4);
        
        % Calculate position error (Euclidean distance)
        position_error = norm([px - px_sol, py - py_sol, pz - pz_sol]);
        
        % Calculate joint angle error with wrapping
        angle_diff = test_theta - sol_deg;
        angle_diff_wrapped = wrapTo180(angle_diff);
        angle_error = norm(angle_diff_wrapped);
        
        % Track best solution
        if position_error < min_error
            min_error = position_error;
            best_solution = sol_deg;
            best_position_error = position_error;
            best_angle_error = angle_error;
        end
    end

    min_error
    best_solution
    best_angle_error
    best_position_error
end

disp("-----")

% Test inverse kinematics by round-trip verification
count_possible = 0;
count_iteration = 0;
position_errors = [];
solution_errors = [];

% Joint angle ranges (in degrees)
t1_range = -180:5:180;    % Shoulder rotation
t2_range = -180:5:180;    % Upper arm flexion
t3_range = -180:5:180;  % Forearm flexion
num_samples = numel(t1_range)*numel(t2_range)*numel(t3_range);

fprintf('Testing %d configurations...\n', num_samples);

for t1 = t1_range
    for t2 = t2_range
        for t3 = t3_range
            set_angle = [t1, t2, t3];
            count_iteration = count_iteration + 1;
            
            % Compute target position
            T_target = BaseToTool(set_angle(1), set_angle(2), set_angle(3));
            px_target = T_target(1,4); 
            py_target = T_target(2,4); 
            pz_target = T_target(3,4);
            
            % Get all IK solutions
            all_solutions = inverseKinematics(px_target, py_target, pz_target);
            
            if ~isempty(all_solutions)
                count_possible = count_possible + 1;
                
                % Initialize best solution tracking
                min_error = Inf;
                best_solution = [];
                best_position_error = Inf;
                best_angle_error = Inf;
                
                % Evaluate each solution
                for i = 1:size(all_solutions,1)
                    % Convert to degrees for comparison
                    sol_deg = rad2deg(all_solutions(i,:));
                    
                    % Compute forward kinematics for this solution
                    T_solution = BaseToTool(sol_deg(1), sol_deg(2), sol_deg(3));
                    px_sol = T_solution(1,4);
                    py_sol = T_solution(2,4);
                    pz_sol = T_solution(3,4);
                    
                    % Calculate position error (Euclidean distance)
                    position_error = norm([px_target - px_sol, py_target - py_sol, pz_target - pz_sol]);
                    
                    % Calculate joint angle error with wrapping
                    angle_diff = set_angle - sol_deg;
                    angle_diff_wrapped = wrapTo180(angle_diff);
                    angle_error = norm(angle_diff_wrapped);
                    
                    % Track best solution
                    if position_error < min_error
                        min_error = position_error;
                        best_solution = sol_deg;
                        best_position_error = position_error;
                        best_angle_error = angle_error;
                    end
                end
                
                % Store errors for analysis
                position_errors(end+1) = best_position_error;
                solution_errors(end+1) = best_angle_error;
            % else
            %     fprintf("Failure angle set: %.4f, %.4f, %.4f \n", t1, t2, t3);
            %     fprintf("Failure target: %.4f, %.4f, %.4f \n", px_target, py_target, pz_target);
            end
        end
    end
end
disp("-----")

% Summary statistics
fprintf('\nTested %d configurations\n', count_iteration);
fprintf('Found solutions for %d configurations (%.1f%%)\n', ...
        count_possible, 100*count_possible/count_iteration);
fprintf('Mean position error: %.6f mm\n', mean(position_errors));
fprintf('Max position error: %.6f mm\n', max(position_errors));
fprintf('Mean angle error: %.2f degrees\n', mean(solution_errors));
fprintf('Max angle error: %.2f degrees\n', max(solution_errors));

% Plot position errors
figure;
histogram(position_errors, 50);
title('Position Errors Distribution');
xlabel('Error (mm)');
ylabel('Frequency');
grid on;

% Plot angle errors
figure;
histogram(solution_errors, 50);
title('Angle Errors Distribution');
xlabel('Error (degrees)');
ylabel('Frequency');
grid on;

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

function solutions = inverseKinematics(px, py, pz)
    constant_y = 0.26451;
    d12x = 1.3; d12z = 95;
    d23x = -133.3; d23z = 0.5;
    d3eex = -126.994; d3eez = 2.8614;

    S = 126.994^2 + 2.8614^2;
    K0 = 133.3^2 + 0.5^2 + S;
    K1 = 2 * (133.3*126.994 + 0.5*2.8614);
    K2 = 2 * (-133.3*2.8614 + 0.5*126.994);

    r = sqrt(px^2 + py^2);
    solutions = [];

    if abs(constant_y) > r
        return; 
    end

    phi = atan2(px, py);
    gamma = acos(constant_y / r);
    theta1_candidates = [-phi + gamma; -phi - gamma];

    for theta1 = theta1_candidates'
        a = cos(theta1);
        b = sin(theta1);
        W = a*px + b*py;
        M = W - d12x;
        N = pz - d12z;
        R_val = M^2 + N^2;

        RHS = R_val - K0;
        normK = sqrt(K1^2 + K2^2);
        if abs(RHS) > normK
            continue;
        end
        alpha = atan2(K2, K1);
        beta = acos(RHS / normK);
        theta3_candidates = [alpha + beta; alpha - beta];

        for theta3 = theta3_candidates'
            U = cos(theta3);
            V = sin(theta3);
            A = d23x + d3eex*U + d3eez*V;
            B = d23z - d3eex*V + d3eez*U;
            denom = A^2 + B^2;

            if denom < 1e-6
                continue;
            end
            cos_theta2 = (A*M + B*N) / denom;
            sin_theta2 = (B*M - A*N) / denom;
            theta2 = atan2(sin_theta2, cos_theta2);

            solutions = [solutions; theta1, theta2, theta3];
        end
    end
end
