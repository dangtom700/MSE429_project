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

    fprintf("Minimum error: %.4f\n",min_error);
    fprintf("Minimum angle error: %.4f\n",best_angle_error);
    fprintf("Minimum position error: %.4f\n",best_position_error);
    fprintf("Best angle solutions: %.4f, %.4f, %.4f\n", best_solution);
end

disp("-----")

% Test inverse kinematics by round-trip verification
count_possible = 0;
count_iteration = 0;

% Joint angle ranges (in degrees)
t1_range = -180:5:180;    % Shoulder rotation
t2_range = -180:5:180;    % Upper arm flexion
t3_range = -180:5:180;  % Forearm flexion
num_samples = numel(t1_range)*numel(t2_range)*numel(t3_range);
position_error = zeros(1,num_samples);
solution_errors = zeros(1,num_samples);

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
                position_errors(count_iteration) = best_position_error;
                solution_errors(count_iteration) = best_angle_error;
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
    % Convert degrees to radians
    t1 = deg2rad(theta1_deg);
    t2 = deg2rad(theta2_deg);
    t3 = deg2rad(theta3_deg);
    
    % Precompute sines and cosines
    C1 = cos(t1); S1 = sin(t1);
    C2 = cos(t2); S2 = sin(t2);
    
    C23 = cos(t2 + t3);
    S23 = sin(t2 + t3);
    
    % Position equations (from ^0_{ee}T)
    x = C1*(2.8614*S23 - 126.994*C23 - 133.3*C2 + 0.5*S2 + 1.3) - 0.2645*S1;
    y = S1*(2.8614*S23 - 126.994*C23 - 133.3*C2 + 0.5*S2 + 1.3) + 0.2645*C1;
    z = 126.994*S23 + 2.8614*C23 + 0.5*C2 + 133.3*S2 + 95;
    
    % Rotation matrix (simplified from ^0_{ee}T)
    R = [C1*C23, -S1, C1*S23;
         S1*C23,  C1, S1*S23;
         -S23,     0, C23];

    % Assemble homogeneous transformation matrix
    T = [R, [x; y; z];
         0, 0, 0, 1];
end

function solutions = inverseKinematics(px, py, pz)
    R = 2.8614^2 + 126.994^2;
    C = 133.3^2 + 0.5^2;

    solutions = [];
    
    r_squared = px^2 + py^2;
    if r_squared < 0.2645^2
        return;  % No valid theta1
    end
    
    alpha = atan2(px, -py);
    beta = acos(-0.2645 / sqrt(r_squared));
    theta1_candidates = [alpha + beta, alpha - beta];

    for t1 = theta1_candidates
        c1 = cos(t1); s1 = sin(t1);
        U = c1*px + s1*py - 1.3;
        V = pz - 95;
        M = 133.3*U - 0.5*V;
        N = -0.5*U - 133.3*V;
        L = 0.5 * (R - C - U^2 - V^2);

        if M^2 + N^2 < L^2
            continue;  % No real solution for theta2
        end
        
        alpha2 = atan2(N, M);
        beta2 = acos(L / sqrt(M^2 + N^2));
        theta2_candidates = [alpha2 + beta2, alpha2 - beta2];

        for t2 = theta2_candidates
            c2 = cos(t2); s2 = sin(t2);
            P = U + 133.3*c2 - 0.5*s2;
            Q = V - 0.5*c2 - 133.3*s2;

            s23 = (2.8614*P + 126.994*Q) / R;
            c23 = (-126.994*P + 2.8614*Q) / R;
            t3 = atan2(s23, c23) - t2;

            % Convert to degrees for output
            solutions = [solutions;
                         rad2deg(t1), rad2deg(t2), rad2deg(t3)];
        end
    end

    solutions = deg2rad(solutions);
end