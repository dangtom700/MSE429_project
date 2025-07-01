%{
Overview:
The code is created to find out the setup of DH parameters found in a full
transformation matrix from base to end point. There are many variable to
tune to when the full homogeneous transformation matrix does not follow
exact same way in the code, especially when assembling components together
require some oddly specify to make the entirely looks assembled.

Process:
Given the fact that there are 4 link (including the end point). There are
in total 16 parameters to tune. The algorithm to compute the DH
transformation is provided
%}

%% MAIN CODE
clc;
sample_data = rand(50,3) * 360 - 180;  % 50 test cases

% Precompute true transformation matrices for all test cases
true_transforms = cell(size(sample_data,1), 1);
for i = 1:size(sample_data,1)
    angles_deg = sample_data(i,:);
    true_transforms{i} = BaseToTool(angles_deg(1), angles_deg(2), angles_deg(3));
end

% Define bounds for the 12 parameters: [d(1:4), a(1:4), alpha(1:4)]
lb = [-300, -300, -300, -300, -300, -300, -300, -300, -pi, -pi, -pi, -pi];
ub = [300, 300, 300, 300, 300, 300, 300, 300, pi, pi, pi, pi];

% PSO parameters
n_particles = 100;
n_dims = 12;
max_iter = 10000;
threshold = 0.1;  % Error threshold for termination
display_interval = 500;  % Display progress every N iterations

% Initialize particles and velocities
X = repmat(lb, n_particles, 1) + rand(n_particles, n_dims) .* (repmat(ub - lb, n_particles, 1));
V = zeros(n_particles, n_dims);

% Initialize personal best and global best
pbest = X;
pbest_val = inf(1, n_particles);
gbest = zeros(1, n_dims);
gbest_val = inf;

% Evaluate initial particles using precomputed transforms
for i = 1:n_particles
    f = objective_function(X(i, :), sample_data, true_transforms);
    pbest_val(i) = f;
    if f < gbest_val
        gbest_val = f;
        gbest = X(i, :);
    end
end

% Display initial best error
fprintf('Iteration 0: Best Error = %f\n', gbest_val);

% PSO constants
w = 1; % Controls the momentum of the particles
c1 = 2.5; % Attraction to particle's personal best
c2 = 1.5; % Attraction to swarm's global best

% Velocity limits
v_max = 0.05 * (ub - lb);

% Initialize iteration counter
iter = 0;

% PSO main loop
while gbest_val > threshold && iter < max_iter
    iter = iter + 1;
    
    for i = 1:n_particles
        % Update velocity
        r1 = rand();
        r2 = rand();
        V(i, :) = w * V(i, :) + c1 * r1 * (pbest(i, :) - X(i, :)) + c2 * r2 * (gbest - X(i, :));
        
        % Clip velocity
        V(i, :) = min(max(V(i, :), -v_max), v_max);
        
        % Update position
        X(i, :) = X(i, :) + V(i, :);
        
        % Enforce bounds
        X(i, :) = min(max(X(i, :), lb), ub);
        
        % Evaluate using pre-computed transforms
        f = objective_function(X(i, :), sample_data, true_transforms);
        
        % Update personal best
        if f < pbest_val(i)
            pbest_val(i) = f;
            pbest(i, :) = X(i, :);
        end
        
        % Update global best
        if f < gbest_val
            gbest_val = f;
            gbest = X(i, :);
        end
    end
    
    % Display progress
    if mod(iter, display_interval) == 0
        fprintf('Iteration %d: Best Error = %f\n', iter, gbest_val);
        d_params = gbest(1:4);
        a_params = gbest(5:8);
        alpha_params = gbest(9:12);
        fprintf('d: [%f, %f, %f, %f]\n', d_params);
        fprintf('a: [%f, %f, %f, %f]\n', a_params);
        fprintf('alpha: [%f, %f, %f, %f] deg\n', rad2deg(alpha_params));
        fprintf('------------------------------------\n');
    end
end

% Display termination reason
if gbest_val <= threshold
    fprintf('\nThreshold reached at iteration %d!', iter);
    fprintf(' Current error (%f) <= threshold (%f)\n', gbest_val, threshold);
else
    fprintf('\nMaximum iterations reached (%d). Best error: %f\n', iter, gbest_val);
end

% Output the best parameters
fprintf('\nFinal results after %d iterations:\n', iter);
d_params = gbest(1:4);
a_params = gbest(5:8);
alpha_params = gbest(9:12);
fprintf('d: [%f, %f, %f, %f]\n', d_params);
fprintf('a: [%f, %f, %f, %f]\n', a_params);
fprintf('alpha: [%f, %f, %f, %f] rad\n', alpha_params);
fprintf('Best error: %f\n', gbest_val);

%% FUNCTIONS
function T = dh_transform(theta, d, a, alpha)
    n = length(theta);
    T = eye(4);
    for i = 1:n
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        Ti = [ct, -st*ca, st*sa, a(i)*ct;
              st, ct*ca, -ct*sa, a(i)*st;
              0, sa, ca, d(i);
              0, 0, 0, 1];
        T = T * Ti;
    end
end

function T = BaseToTool(theta1_deg, theta2_deg, theta3_deg)
    t1 = deg2rad(theta1_deg);
    t2 = deg2rad(theta2_deg);
    t3 = deg2rad(theta3_deg);
    C1 = cos(t1); S1 = sin(t1);
    C2 = cos(t2); S2 = sin(t2);
    C23 = cos(t2 + t3);
    S23 = sin(t2 + t3);
    x = C1*(2.8614*S23 - 126.994*C23 - 133.3*C2 + 0.5*S2 + 1.3) - 0.2645*S1;
    y = S1*(2.8614*S23 - 126.994*C23 - 133.3*C2 + 0.5*S2 + 1.3) + 0.2645*C1;
    z = 126.994*S23 + 2.8614*C23 + 0.5*C2 + 133.3*S2 + 95;
    R = [C1*C23, -S1, C1*S23;
         S1*C23, C1, S1*S23;
         -S23, 0, C23];
    T = [R, [x; y; z]; 0, 0, 0, 1];
end

% Modified objective function with pre-computed transforms
function avg_error = objective_function(params, test_cases, true_transforms)
    d = params(1:4);
    a = params(5:8);
    alpha = params(9:12);
    total_error = 0;
    num_test_case = size(test_cases, 1);
    
    for i = 1:num_test_case
        angles_deg = test_cases(i, :);
        T_true = true_transforms{i};  % Use pre-computed true transform
        theta_rad = deg2rad(angles_deg);
        theta = [theta_rad, 0];  % 4th joint angle fixed at 0
        T_candidate = dh_transform(theta, d, a, alpha);
        diff = norm(T_true(1:3,4) - T_candidate(1:3,4));
        total_error = total_error + diff;
    end

    avg_error = total_error / num_test_case;
end