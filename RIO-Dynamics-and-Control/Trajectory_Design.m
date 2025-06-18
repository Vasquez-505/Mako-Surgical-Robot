%% TASK SPACE TRAJECTORY DESIGN

% This script generates a smooth joint-space trajectory that moves the robot's
% end-effector from an initial position and orientation (pA, R_A) to a final
% position and orientation (pB, R_B) over a fixed duration (T seconds). The
% trajectory is computed by first solving inverse kinematics (using CLICK)
% to obtain the start and end joint configurations (qA, qB), and then using
% cubic polynomial interpolation to compute joint positions, velocities, and
% accelerations at each timestep. The resulting trajectory is formatted as
% time series data compatible with Simulink for use with controllers such
% as PID or Computed Torque Control. It also includes visualization plots
% for analysis and debugging.


% ======= PARAMETERS =======
nJoints = 6;         % Number of joints
T = 5;               % Total movement time in seconds
dt = 0.01;           % Time step (100Hz sampling)
t = 0:dt:T;          % Time vector

% ======= START AND END JOINT POSITIONS (rad or m) =======


pA = [0.2; 0.1; 0.2]; % initial position [rad]
R_A = eye(3);         % initial orientation
pB = [0.6; 0.1; 0.2]; % final position [rad]
R_B = eye(3);         % final orientation

% CLICK Parameters for calculating q for a given desired position &
% orientation
q0 = [0; 0; 0; 0; 0; 0];
d_t = 0.01;
tol = 0.0001;
max_iter = 500;

% Replace these with your IK results:
qA = CLICK(q0, pA, R_A, dt, tol, max_iter)% start joint angles
qB = CLICK(q0, pB, R_B, dt, tol, max_iter) % end joint angles

% ======= PREALLOCATE COEFFICIENTS =======
a0 = zeros(nJoints,1);
a1 = zeros(nJoints,1);
a2 = zeros(nJoints,1);
a3 = zeros(nJoints,1);

% ======= COMPUTE CUBIC COEFFICIENTS FOR EACH JOINT =======
for i = 1:nJoints
    a0(i) = qA(i);
    a1(i) = 0; % initial velocity zero
    
    % Setup linear system for a2 and a3
    A = [T^2, T^3;
         2*T, 3*T^2];
    b = [qB(i) - qA(i);
         0]; % final velocity zero
    
    x = A\b;
    
    a2(i) = x(1);
    a3(i) = x(2);
end

% ======= CALCULATE TRAJECTORIES =======
q = zeros(nJoints, length(t));
dq = zeros(nJoints, length(t));
ddq = zeros(nJoints, length(t));

for i = 1:nJoints
    q(i,:) = a0(i) + a1(i)*t + a2(i)*t.^2 + a3(i)*t.^3;
    dq(i,:) = a1(i) + 2*a2(i)*t + 3*a3(i)*t.^2;
    ddq(i,:) = 2*a2(i) + 6*a3(i)*t;
end

% ======= FORMAT FOR SIMULINK =======
% Columns: time, q1, q2, ..., q6
qd_simulink = [t' q'];
dqd_simulink = [t' dq'];
ddqd_simulink = [t' ddq'];

qd_ts = timeseries(qd_simulink(:,2:end), qd_simulink(:,1));
dqd_ts = timeseries(dqd_simulink(:,2:end), dqd_simulink(:,1));
ddqd_ts = timeseries(ddqd_simulink(:,2:end), ddqd_simulink(:,1));

% ======= Save to file or workspace =======
save('precomputed_trajectory.mat', 'qd_ts', 'dqd_ts', 'ddqd_ts');


%% ======= PLOT FOR VISUALIZATION =======
figure;
for i = 1:nJoints
    subplot(nJoints,1,i)
    plot(t, q(i,:), 'b', t, dq(i,:), 'r', t, ddq(i,:), 'g')
    xlabel('Time [s]')
    ylabel(['Joint ', num2str(i)])
    legend({'Position', 'Velocity', 'Acceleration'})
    grid on
end
sgtitle('Joint Trajectories (Position, Velocity, Acceleration)')

%% ======= SEPARATE JOINTS PLOT FOR VISUALIZATION =======
for i = 1:nJoints
    figure; % new figure for each joint
    plot(t, q(i,:), 'b', 'LineWidth', 1.5)
    hold on
    plot(t, dq(i,:), 'r', 'LineWidth', 1.5)
    plot(t, ddq(i,:), 'g', 'LineWidth', 1.5)
    hold off
    xlabel('Time [s]')
    ylabel(['Joint ', num2str(i)])
    legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best')
    title(['Joint ', num2str(i), ' Trajectories'])
    grid on
end