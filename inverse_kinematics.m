clc;
clear;
close all;

% Test inverse kinematics by round-trip verification
count_possible = 0;
count_iteration = 0;
position_errors = [];
solution_errors = [];

% Joint angle ranges (in degrees)
t1_range = 0:5:270;    % Shoulder rotation
t2_range = 0:5:135;    % Upper arm flexion
t3_range = -45:5:135;  % Forearm flexion

fprintf('Testing %d configurations...\n', numel(t1_range)*numel(t2_range)*numel(t3_range));

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
                
                % Display progress
                if mod(count_possible, 1000) == 0
                    fprintf('Tested %d valid configurations\n', count_possible);
                end
            end
        end
    end
end

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
         s1*c23,  c1, -s1*s23, py;
             s23,   0,      c23, pz;
               0,   0,        0,  1];
end

function solutions = inverseKinematics(px, py, pz)
    solutions = [];
    denom = px^2 + py^2;
    
    % Check if solution for theta1 exists
    if denom < 0.25
        return;
    end
    
    % Step 1: Solve for theta1 (two candidates)
    sqrt_term = sqrt(denom - 0.25);
    
    % Candidate 1: Use '+' branch
    c1_1 = (-0.5*py + px*sqrt_term) / denom;
    s1_1 = (0.5*px + py*sqrt_term) / denom;
    theta1_1 = atan2(s1_1, c1_1);
    
    % Candidate 2: Use '-' branch
    c1_2 = (-0.5*py - px*sqrt_term) / denom;
    s1_2 = (0.5*px - py*sqrt_term) / denom;
    theta1_2 = atan2(s1_2, c1_2);
    
    % Process each theta1 candidate
    for theta1 = [theta1_1, theta1_2]
        c1 = cos(theta1);
        s1 = sin(theta1);
        
        % Compute intermediate terms
        A = c1*px + s1*py;  % Represents 40 + 140*cos(theta2) + 125*cos(theta23)
        C = pz - 95;        % Represents 140*sin(theta2) + 125*sin(theta23)
        
        % Step 2: Solve for theta3
        numerator = (A - 40)^2 + C^2 - 140^2 - 125^2;
        denominator = 2 * 140 * 125;
        cos_theta3 = numerator / denominator;
        
        if abs(cos_theta3) > 1
            continue; % Skip invalid theta3
        end
        
        % Two solutions for theta3 (elbow up/down)
        theta3_1 = acos(cos_theta3);
        theta3_2 = -theta3_1;
        
        for theta3 = [theta3_1, theta3_2]
            c3 = cos(theta3);
            s3 = sin(theta3);
            
            % Compute coefficients for theta2 equation
            P = 140 + 125*c3;
            Q = 125*s3;
            denom2 = P^2 + Q^2;
            
            if denom2 < 1e-6
                continue; % Avoid division by zero
            end
            
            % Step 3: Solve for theta2
            cos_theta2 = (P*(A - 40) + Q*C) / denom2;
            sin_theta2 = (P*C - Q*(A - 40)) / denom2;
            
            % Normalize to handle numerical errors
            norm_val = sqrt(cos_theta2^2 + sin_theta2^2);
            if norm_val > 0
                cos_theta2 = cos_theta2 / norm_val;
                sin_theta2 = sin_theta2 / norm_val;
            end
            
            theta2 = atan2(sin_theta2, cos_theta2);
            
            % Store valid solution
            solutions = [solutions; theta1, theta2, theta3];
        end
    end
end

function angle = wrapTo180(angle)
    % Wrap angle to [-180,180] range
    angle = mod(angle + 180, 360) - 180;
end