%% Generate Matlab and Simulink Code for RobotX
%  Create Simulink Library file RobotX_Lib.mdl
new_system('RIO_Lib','Library');
open_system('RIO_Lib');

%% Direct Kinematics and Jacobian
[RIO_T, J] = DKin(RIO);

RIO_R = RIO_T(1:3, 1:3);
RIO_p = RIO_T(1:3, 4);

matlabFunctionBlock('RIO_Lib/RIO_Direct_Kinematics', RIO_R, RIO_p, J);

%% Save library in current Directory
save_system('RIO_Lib');
close_system('RIO_Lib');

%% Create vrrobot Block
vrrobot(RIO)