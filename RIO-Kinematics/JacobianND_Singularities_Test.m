%% Validation of the Analytical Jacobian using Numerical Differentiation

% Define symbolic joint variables
syms q1 q2 q3 q4 q5 q6 real

Robot = RIO();  

% Get symbolic direct kinematics and Jacobian
[T_sym, J_sym] = DKin(Robot);  % T_sym: 4x4, J_sym: 6x6

% Set test joint configuration (radians)
q_vals = [0, 0, 0, 0, 0, 0];  %  Match values used in Simulink to verify


% Analytical Jacobian at that configuration
J_analytical = double(subs(J_sym, [q1 q2 q3 q4 q5 q6], q_vals));

% Finite difference for full Jacobian (linear + angular)
epsilon = 1e-6;
J_numerical = zeros(6,6);

for i = 1:6
    dq = zeros(1,6);
    dq(i) = epsilon;

    q_plus = q_vals + dq;
    q_minus = q_vals - dq;

    % Evaluate T at q+ and q-
    T_plus = double(subs(T_sym, [q1 q2 q3 q4 q5 q6], q_plus));
    T_minus = double(subs(T_sym, [q1 q2 q3 q4 q5 q6], q_minus));

    % ---- Linear Velocity (position derivative) ----
    p_plus = T_plus(1:3, 4);
    p_minus = T_minus(1:3, 4);
    v_i = (p_plus - p_minus) / (2 * epsilon);

    % ---- Angular Velocity (from R_dot * R') ----
    R_plus = T_plus(1:3, 1:3);
    R_minus = T_minus(1:3, 1:3);

    R_dot = (R_plus - R_minus) / (2 * epsilon);
    R_mid = T_plus(1:3,1:3);  % Could also average R_plus and R_minus
    Omega_skew = R_dot * R_mid';

    % Extract angular velocity from skew-symmetric matrix
    w_i = [Omega_skew(3,2); Omega_skew(1,3); Omega_skew(2,1)];

    % Fill i-th column of Jacobian
    J_numerical(:, i) = [v_i; w_i];
end

% Display results

disp('--- Jacobian Finite Difference ---');
disp(J_numerical);
disp('--- Simulink Jacobian ---')
disp(J_analytical)
disp('--- Jacobian Error --- ')
disp(J_analytical - J_numerical)


%% Extra Validation Metrics
% Summary metrics
fprintf('Rank of Analytical Jacobian: %d\n', rank(J_analytical));
fprintf('Condition Number: %.2e\n', cond(J_analytical));


%% Wrist & Arm Singularaties Search

clc; clear;

% Symbolic variables and robot model
syms q1 q2 q3 q4 q5 q6 real
Robot = RIO();  
[~, J_sym] = DKin(Robot);

q_syms = [q1 q2 q3 q4 q5 q6];

% Joint ranges - adjust resolution as needed
q_ranges = {
    linspace(-pi, pi, 10)  % q1
    linspace(-pi, pi, 7)   % q2
    linspace(-pi, pi, 7)   % q3
    linspace(-pi, pi, 5)   % q4
    linspace(-pi, pi, 5)   % q5
    linspace(-pi, pi, 5)   % q6
};

fprintf('Singular configurations detected:\n');
fprintf('   q1     q2     q3     q4     q5     q6     Rank   MinSingVal   CondNum   Type\n');

% Thresholds
threshold_rank = 6;       % full rank expected (6 for 6-DOF)
threshold_cond = 1e6;     % condition number threshold for near-singularity
threshold_min_sv = 1e-3;  % min singular value threshold for near-singularity

for q1_val = q_ranges{1}
    for q2_val = q_ranges{2}
        for q3_val = q_ranges{3}
            for q4_val = q_ranges{4}
                for q5_val = q_ranges{5}
                    for q6_val = q_ranges{6}
                        q_vals = [q1_val, q2_val, q3_val, q4_val, q5_val, q6_val];
                        J_num = double(subs(J_sym, q_syms, q_vals));
                        
                        % Full Jacobian rank and condition number
                        rnk = rank(J_num, 1e-3);
                        cnum = cond(J_num);
                        
                        % Minimum singular value of full Jacobian
                        s = svd(J_num);
                        min_sv = min(s);
                        
                        % Jacobian partitions by columns
                        J_arm = J_num(:,1:3);
                        J_wrist = J_num(:,4:6);
                        
                        rank_arm = rank(J_arm, 1e-3);
                        rank_wrist = rank(J_wrist, 1e-3);
                        
                        % Detect singularities by rank, cond or min singular value
                        if (rnk < threshold_rank) || (cnum > threshold_cond) || (min_sv < threshold_min_sv)
                            if rank_arm < 3
                                sing_type = 'Arm';
                            elseif rank_wrist < 3
                                sing_type = 'Wrist';
                            else
                                sing_type = 'General';
                            end
                            
                            fprintf('%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f     %d       %.2e   %.2e    %s\n', ...
                                q_vals, rnk, min_sv, cnum, sing_type);
                        end
                    end
                end
            end
        end
    end
end


%% Arm Singularities Search

% Symbolic variables
syms q1 q2 q3 q4 q5 q6 real
q = [q1 q2 q3 q4 q5 q6];

% Robot DH Parameters 
Robot = RIO()

% Compute Jacobian symbolically once
[~, J_sym] = DKin(Robot);

% Joint ranges and resolution
nPts = 15;
q_range = linspace(-pi, pi, nPts);

% Fixed wrist joints
fixed_wrist = [0 0 0];

singular_arm = [];
threshold = 1e-3; % Singular value threshold

total_points = nPts^3;
count = 0;

min_singular_value = inf;

for q1v = q_range
for q2v = q_range
for q3v = q_range
    count = count + 1;
    % Substitute joint values with wrist fixed
    J_num = double(subs(J_sym, q, [q1v q2v q3v fixed_wrist]));
    
    J_arm = J_num(:,1:3);
    s = svd(J_arm);
    min_sv = min(s);
    if min_sv < min_singular_value
        min_singular_value = min_sv;
    end
    
    if min_sv < threshold
        fprintf('Arm singularity near q = [%f, %f, %f], min singular value = %g\n', q1v, q2v, q3v, min_sv);
        singular_arm = [singular_arm; q1v q2v q3v];
    end
end
end
end

fprintf('Minimum singular value encountered: %g\n', min_singular_value);
fprintf('Total arm singularities detected: %d\n', size(singular_arm,1));

% Plot if any found
if ~isempty(singular_arm)
    figure; hold on; grid on;
    scatter3(singular_arm(:,1), singular_arm(:,2), singular_arm(:,3), 'ro');
    title('Arm Singularities (q1, q2, q3)');
    xlabel('q1'); ylabel('q2'); zlabel('q3');
else
    disp('No arm singularities detected in sampled range.');
end

%% Wrist Singularities Search

% Symbolic variables
syms q1 q2 q3 q4 q5 q6 real
q = [q1 q2 q3 q4 q5 q6];

% Robot DH parameters
Robot = RIO()

% Compute symbolic Jacobian
[~, J_sym] = DKin(Robot);

% Joint ranges and resolution
nPts = 15;
q_range = linspace(-pi, pi, nPts);

% Fix arm joints q1,q2,q3 to zero
fixed_arm = [0 0 0];

singular_wrist = [];
threshold = 1e-3;

total_points = nPts^3;
count = 0;
min_singular_value = inf;

% Loop over wrist joints q4,q5,q6 only
for q4v = q_range
for q5v = q_range
for q6v = q_range
    count = count + 1;
    
    % Substitute joint values with arm fixed
    J_num = double(subs(J_sym, q, [fixed_arm q4v q5v q6v]));
    
    % Extract wrist part of Jacobian (columns 4 to 6)
    J_wrist = J_num(:,4:6);
    
    % Compute singular values
    s = svd(J_wrist);
    min_sv = min(s);
    
    if min_sv < min_singular_value
        min_singular_value = min_sv;
    end
    
    if min_sv < threshold
        fprintf('Near-singularity at wrist q = [%f, %f, %f], min singular value = %g\n', q4v, q5v, q6v, min_sv);
        singular_wrist = [singular_wrist; q4v q5v q6v];
    end
end
end
end

fprintf('Minimum singular value encountered (wrist): %g\n', min_singular_value);
fprintf('Total near-singular wrist configurations found: %d\n', size(singular_wrist,1));

% Plot wrist singularities if any found
if ~isempty(singular_wrist)
    figure; hold on; grid on;
    scatter3(singular_wrist(:,1), singular_wrist(:,2), singular_wrist(:,3), 'bs');
    title('Wrist Singularities (q4, q5, q6)');
    xlabel('q4 (rad)'); ylabel('q5 (rad)'); zlabel('q6 (rad)');
else
    disp('No wrist singularities detected in the sampled range.');
end
