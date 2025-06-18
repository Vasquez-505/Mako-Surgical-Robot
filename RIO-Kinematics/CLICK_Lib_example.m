%% Step 1: Create the Simulink Library
new_system('CLICK_Lib','Library');
open_system('CLICK_Lib');

%% Step 2: Add a MATLAB Function Block
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    'CLICK_Lib/CLICK_Block');

%% Step 3: Open the block and paste in this code:
% function q_out = fcn(q0, pd, Rd, dt, tol, max_iter)
%     %#codegen
%     coder.extrinsic('CLICK');
% 
%     q_out = zeros(6,1); % Predefine output size for Simulink
% 
%     q_out = CLICK(q0, pd, Rd, dt, tol, max_iter);
% end


%% Step 4: Save and close 
save_system('CLICK_Lib');
close_system('CLICK_Lib');
 

%% CLICK Function Implementation Testing 

% Initial joint configuration
q0 = zeros(6,1);

% Desired end-effector position
pd = [0.4; 0.1; 0.2];

% Desired orientation (RPY in radians)
Rd = [1 0 0;
      0 1 0;
      0 0 1];  

% CLICK controller parameters
dt = 0.01;
tol = 1e-4;
max_iter = 100;

% Run CLICK inverse kinematics
q_result = CLICK(q0, pd, Rd, dt, tol, max_iter);  % <-- FIXED: passing rpy

% Display result
disp('Resulting joint configuration:');
disp(q_result);

%% Validation with Direct Kinematics

% Desired end-effector position and rotation
pd_test = [0.4; 0.1; 0.2];

Rd_test = [1 0 0;
           0 1 0;
           0 0 1]; 

% Initial joint configuration
q0_test = zeros(6,1);

% CLICK controller parameters
dt_test = 0.01;
tol_test = 1e-4;
max_iter_test = 300; %NOTE: Around 300 iterations the error is low

% Compute IK
q_ik = CLICK(q0_test, pd_test, Rd_test, dt_test, tol_test, max_iter_test);

% Forward Kinematics function assumed to be available:
[T, ~] = DKin(RIO());
q_test = q_ik; % your IK result
T_eval = double(subs(T, sym('q', [6,1]), q_test));

pos_fk = T_eval(1:3,4);
rot_fk = T_eval(1:3,1:3);

disp('FK Position:');
disp(pos_fk);

disp('FK Rotation:');
disp(rot_fk);

disp('Desired Position:');
disp(pd_test);

disp('Desired Rotation:');
disp(Rd_test);

position_error = norm(pos_fk - pd_test);
orientation_error = norm(rot_fk - Rd_test, 'fro'); % Frobenius norm for matrices

fprintf('Position error norm: %.5f\n', position_error);
fprintf('Orientation error norm: %.5f\n', orientation_error);


