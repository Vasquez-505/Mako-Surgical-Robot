%% Step 1: Create the Simulink library
new_system('ClosedFormIK_Lib','Library');
open_system('ClosedFormIK_Lib');

%% Step 2: Add a MATLAB Function Block
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    'ClosedFormIK_Lib/ClosedFormIK_Block');

%% Step 3: Open the block and paste in this code:
% function q_out = fcn(pd, Rd)
%   %#codegen
%   coder.extrinsic('ClosedFormIK');
% 
%   % Preallocate output
%   q_out = zeros(6,1);
% 
%   % Call your closed-form IK function
%   q_out = ClosedFormIK(pd, Rd);
% 
%   end

%% Step 4: Save and close
save_system('ClosedFormIK_Lib');
close_system('ClosedFormIK_Lib');

%% Closed-Form IK Outpt Test 

% Desired end-effector position
pd = [0.4; 0.1; 0.2];

% Desired orientation rotation matrix 
Rd = [1 0 0; 
      0 1 0; 
      0 0 1];

% Compute IK (joint angles)
try
    q_ik = ClosedFormIK(pd, Rd);
    disp("Joint angles from IK (radians):");
    disp(q_ik');
catch ME
    disp("Error during inverse kinematics:");
    disp(ME.message);
    return
end

%% Validation with Direct Kinematics

% Desired end-effector position and rotation
pd_test = [0.4; 0.1; 0.2];
Rd_test = eye(3);

% Compute IK
q_ik = ClosedFormIK(pd_test, Rd_test);

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