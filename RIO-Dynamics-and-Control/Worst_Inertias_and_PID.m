%% Inertias Matrix Numeric Function 

syms q [6 1] real

M = S_inertia_matrix(q);
matlabFunction(M, 'Vars', {q}, 'File', 'Numeric_inertia_matrix');

%% Worst Inertias Evaluation
q_worst_inertia = [0; 0; 0; pi/2; 0; pi/2]; % possible configuration arm stretched

M_worst_inertia = Numeric_inertia_matrix(q_worst_inertia)


%% Worst-Case Inertia Values Found

I_worst = [1.9891 ,1.9427 , 0.1584, 0.4638, 0.0791, 0.0031]
I1 = I_worst(1);
I2 = I_worst(2);
I3 = I_worst(3);
I4 = I_worst(4);
I5 = I_worst(5);
I6 = I_worst(6);


%% PID tunning gain values

w_n = 6;
zeta = 0.7;

% Initial Aproximation
Kp = w_n^2 * I_worst
Ki = 0.1 * Kp
Kd = 2 * zeta * w_n * I_worst
N = 40 * w_n

%% Final parameter Values

w_n = 6;
zeta = 0.7;

Kp = 3 * w_n^2 * I_worst
Ki = 0.1 * Kp
Kd = 9 * zeta * w_n * I_worst
N = 40*w_n