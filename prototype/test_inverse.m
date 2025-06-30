clc;

% Test with target from original code
target_angles = [75, 10, -20];
T = BaseToEnd(target_angles(1), target_angles(2), target_angles(3));
x = T(1,4); y = T(2,4); z = T(3,4);

fprintf("Location: (%.4f, %.4f, %.4f)\n", x, y, z)
% Compute inverse kinematics
solutions = inverseKinematics(x, y, z);
disp('Valid Solutions (deg):');
disp(solutions);

function solutions = inverseKinematics(px, py, pz)
    % Link parameters (mm)
    A = 6.78;         % a1
    B = 133.794;      % a2
    C = 112.268;      % a3 (end effector)
    
    solutions = [];   % Initialize solution matrix
    
    % ===== Step 1: Solve for θ₁ =====
    theta1_deg = rad2deg(atan2(py, px));
    
    % Adjust θ₁ to [0°, 360°) range
    if theta1_deg < 0
        theta1_deg = theta1_deg + 360;
    end
    
    % Check θ₁ joint limits [0°, 270°]
    if theta1_deg < 0 || theta1_deg > 270
        return;  % No valid solution if θ₁ out of bounds
    end
    
    % ===== Step 2: Solve for θ₂ and θ₃ =====
    r = sqrt(px^2 + py^2);  % Radial distance in XY-plane
    r_prime = r - A;
    
    % Intermediate terms
    K = (r_prime^2 + pz^2 + B^2 - C^2) / (2 * B);
    R = sqrt(r_prime^2 + pz^2);
    
    % Check reachability
    if abs(K) > R
        return;  % Point outside workspace
    end
    
    % Base angle for θ₂ solutions
    alpha = atan2(pz, r_prime);
    beta = acos(K / R);
    
    % Two candidate solutions for θ₂ (elbow up/down)
    theta2_rad_candidates = [alpha + beta; alpha - beta];
    
    for k = 1:2
        theta2_rad = theta2_rad_candidates(k);
        theta2_deg = rad2deg(theta2_rad);
        
        % Check θ₂ joint limits [-30°, 90°]
        if theta2_deg < -30 || theta2_deg > 90
            continue;
        end
        
        % Solve for θ₃
        cos_phi = (r_prime - B * cos(theta2_rad)) / C;
        sin_phi = (pz - B * sin(theta2_rad)) / C;
        
        % Prevent numerical errors
        cos_phi = max(min(cos_phi, 1), -1);
        sin_phi = max(min(sin_phi, 1), -1);
        
        phi_rad = atan2(sin_phi, cos_phi);  % θ₂ + θ₃
        theta3_rad = phi_rad - theta2_rad;
        theta3_deg = rad2deg(theta3_rad);
        
        % Check θ₃ joint limits [-180°, 0°]
        if theta3_deg < -180 || theta3_deg > 0
            continue;
        end
        
        % Store valid solution
        solutions = [solutions; theta1_deg, theta2_deg, theta3_deg];
    end
end

function T = BaseToEnd(t1, t2, t3)
    c1 = cosd(t1); s1 = sind(t1); c2=cosd(t2); s2=sind(t2);
    c23 = cosd(t2 + t3); s23 = sind(t2 + t3);
    T = [c1*c23, -c1*s23, s1, c1*(6.78 + 133.784*c2 + 112.268*c23);
         s1*c23, -s1*s23, -c1, s1*(6.78 + 133.784*c2 + 112.268*c23);
         s23, c23, 1, 133.794*s2 + 112.268*s23;
         0,0,0,1];
end