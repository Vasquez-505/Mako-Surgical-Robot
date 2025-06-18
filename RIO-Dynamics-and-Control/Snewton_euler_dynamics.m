function tau = Snewton_euler_dynamics(q, dq, ddq)

% Computes symbolic joint torques using the Newton-Euler method (revolute joints)
% Inputs:  q, dq, ddq - symbolic joint states
% Output:  tau - symbolic joint torques

data = load('mako_robot_mass_properties.mat', 'robot');
robot = data.robot;

DH = RIO();

% Initialize symbolic transformation matrices
T_full = sym(zeros(4,4,6));
T_temp = eye(4);

% New: Store symbolic rotation matrices and position vectors
R_sym = sym(zeros(3,3,6));  % Rotation matrices
P_sym = sym(zeros(3,6));    % Position vectors

% Forward kinematics
for i = 1:6
    A_i = DHTransf(DH(i,:));
    T_temp = simplify(T_temp * A_i);
    T_full(:,:,i) = T_temp;

    % Extract and store rotation and position
    R_sym(:,:,i) = T_temp(1:3, 1:3);
    P_sym(:,i) = T_temp(1:3, 4);
end

% Relative transforms
T = sym(zeros(4,4,6));
T(:,:,1) = T_full(:,:,1);

for i = 2:6
    T(:,:,i) = simplify(T_full(:,:,i-1) \ T_full(:,:,i));
end

% Gravity and base conditions
g0 = [0; 0; 9.81]; % Gravity Force has the same direction as z0 (+)
w0 = [0; 0; 0];
dw0 = [0; 0; 0];
dv0 = g0;
z = [0; 0; 1];

% Initialize symbolic variables
w = sym(zeros(3,6)); dw = w; dv = w; dv_c = w; F = w; N = w; f = w ; n = w;
tau = sym(zeros(6,1));

% Initialize arrays to store relative rotation matrices and position vectors
R_link = sym(zeros(3,3,6));   % Rotation from link i-1 to i
P_link = sym(zeros(3,6));     % Position vector from link i-1 to i

% Forward recursion
for i = 1:6
    R = T(1:3,1:3,i);         % Relative rotation matrix
    P = T(1:3,4,i);           % Relative position vector

    % Store them
    R_link(:,:,i) = R;        %Rotation matrix from i-1 to i
    P_link(:,i) = P;          %Position vector from frame i-1 to i on frame i-1 coordinates
    P_i(:,i) = R_link(:,:,i)' * P_link(:,i);  % Transform positon vector into frame i coordinates


    if i == 1
        w(:,i) = R_link(:,:,i)' * (w0 + dq(i)*z);
        dw(:,i) = R_link(:,:,i)' * (dw0 + ddq(i)*z + cross(w0, dq(i)*z));
        dv(:,i) = R_link(:,:,i)' * dv0 + cross(dw0, P_i(:,i)) + cross(w0, cross(w0, P_i(:,i)));
    else
        w(:,i) = R_link(:,:,i)' * (w(:,i-1) + dq(i)*z);
        dw(:,i) = R_link(:,:,i)' * (dw(:,i-1) + ddq(i)*z + cross(dq(i)*w(:,i-1), z));
        dv(:,i) = R_link(:,:,i)' * dv(:,i-1) + cross(dw(:,i-1), P_i(:,i)) + cross(w(:,i), cross(w(:,i), P_i(:,i)));
    end

    rc = robot(i).CoM;
    dv_c(:,i) = cross(dw(:,i), rc) + cross(w(:,i), cross(w(:,i), rc)) + dv(:,i);
    F(:,i) = robot(i).mass * dv_c(:,i);
    N(:,i) = robot(i).inertia * dw(:,i) + cross(w(:,i), robot(i).inertia * w(:,i));
end


% Backward recursion
for i = 6:-1:1
    rc = robot(i).CoM;
    if i == 6
        f(:,i) = F(:,i);
        n(:,i) = N(:,i) + cross(rc, F(:,i));
    else
        f(:,i) = F(:,i) + R_link(:,:,i+1) * f(:,i+1);
        n(:,i) = N(:,i) +  cross(rc, F(:,i)) + ...
                 R_link(:,:,i+1) * n(:,i+1) + ...
                 cross(-f(:,i),P_i(:,i)+rc);
    end

    % Torque projection along joint axis
    tau(i) = n(:,i)' * R_link(:,:,i)' * z;
end

% Optimize expressions (faster than simplify)
end

