%% Matrixes Functions Generations

syms q1 q2 q3 q4 q5 q6 real
q = [q1; q2; q3; q4; q5; q6];
dq = sym('dq', [6,1], 'real');

[~, C_sym, G_sym] = compute_dynamics(q, dq);

%  Generate and save numeric function files

matlabFunction(C_sym, 'Vars', {q, dq}, 'File', 'Numeric_coriolisMatrix_C');
matlabFunction(G_sym, 'Vars', {q}, 'File', 'Numeric_gravityVector_G');

%% Create Simulink Library

new_system( 'Centralized_I_Lib_Controller', 'Library');
open_system('Centralized_I_Lib_Controller');


%% Add MATLAB Function block for computedTorqueControl


tau = computedTorqueControl_sym
matlabFunctionBlock('Centralized_I_Lib_Controller/computedTorqueControl_sym', tau);

%%

% Export to Simulink block
ddq = directDynamics_sym

matlabFunctionBlock('Centralized_I_Lib_Controller/directDynamics_sym', ...
    ddq, 'Vars', {q, dq, tau});


save_system('Centralized_I_Lib_Controller');
disp(['Simulink library created: ', 'Centralized_I_Lib_Controller', '.slx']);


%% Numeric Functions Matrixes Test

q_test = zeros(6,1);
dq_test = zeros(6,1);

M_val = Numeric_inertia_matrix(q_test)
C_val = Numeric_coriolisMatrix_C(q_test, dq_test)
G_val = Numeric_gravityVector_G(q_test)

%% Test script for computed torque control

% Test inputs
q = zeros(6,1);
dq = zeros(6,1);
qd = [0.1; 0.2; 0.1; 0.3; 0; 0];
dqd = zeros(6,1);
ddqd = zeros(6,1);

% Gains
Kp = diag([100 100 100 50 50 50]);
Kd = diag([20 20 20 10 10 10]);

% Call control law
tau = computedTorqueControl(q, dq, qd, dqd, ddqd, Kp, Kd);

disp('Computed torque:');
disp(tau);

