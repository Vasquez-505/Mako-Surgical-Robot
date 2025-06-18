%% Newton-Euler Numeric Function Generation

% Define symbolic joint variables
syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real

q   = [q1; q2; q3; q4; q5; q6];
dq  = [dq1; dq2; dq3; dq4; dq5; dq6];
ddq = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6];

% Compute symbolic torques
tau = Snewton_euler_dynamics(q, dq, ddq);

% Generate numeric function
matlabFunction(tau, 'Vars', {q, dq, ddq}, 'File', 'newton_euler_numeric');
disp('Generated numeric function: newton_euler_numeric.m');

%% Create Simulink Library 
new_system('NewtonEuler_Lib', 'Library');
open_system('NewtonEuler_Lib');

%% MATLAB Function Block creation/update

syms q1 q2 q3 q4 q5 q6 real
syms dq1 dq2 dq3 dq4 dq5 dq6 real
syms ddq1 ddq2 ddq3 ddq4 ddq5 ddq6 real

q   = [q1; q2; q3; q4; q5; q6];
dq  = [dq1; dq2; dq3; dq4; dq5; dq6];
ddq = [ddq1; ddq2; ddq3; ddq4; ddq5; ddq6];

tau = Snewton_euler_dynamics(q, dq, ddq);

matlabFunctionBlock( 'NewtonEuler_Lib/NewtonEulerTorque', tau,'Optimize', true)                                     
                          
save_system('NewtonEuler_Lib');
disp('Simulink library created: NewtonEuler_Lib.slx');

%% Validation 

% Test disered joint values
q_test   = [0; 0; 0; 0; 0; 0];
dq_test  = [0; 0; 0; 0; 0; 0];
ddq_test = [0; 0; 0; 0; 0; 0];

% Evaluate numeric torque
tau_numeric = newton_euler_numeric (q_test, dq_test, ddq_test);
disp('Computed joint torques (tau):');
disp(tau_numeric);

