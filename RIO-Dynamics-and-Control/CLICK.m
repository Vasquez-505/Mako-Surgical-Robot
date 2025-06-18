%% Click Inverse Kinematics function

function q_out = CLICK(q0, pd, Rd, dt, tol, max_iter)
% CLICK - Cartesian-space inverse kinematics using iterative control
%
% Inputs:
%   q0       : 6x1 initial joint values
%   pd       : 3x1 desired end-effector position
%   Rd       : 3x3 desired end-effector rotation matrix
%   dt       : time step for iteration (e.g., 0.01)
%   tol      : error tolerance for stopping condition (e.g., 1e-4)
%   max_iter : maximum number of iterations (e.g., 100)
%
% Output:
%   q_out    : 6x1 resulting joint vector

    % Initialize robot and symbolic variables
    Robot = RIO();               
    q_syms = sym('q', [6 1]);     

    persistent T_sym J_sym
    if isempty(T_sym)
        [T_sym, J_sym] = DKin(Robot);  % Get symbolic forward kinematics and Jacobian
    end

    q = q0;                                 % Current joint initial values
    Kp = diag([1 * ones(3,1); 1 * ones(3,1)]);  % Proportional gain matrix

    for k = 1:max_iter
        % Evaluate forward kinematics and Jacobian
        T = double(subs(T_sym, q_syms, q));
        J = double(subs(J_sym, q_syms, q));

        % Current position and orientation
        p = T(1:3, 4);
        R = T(1:3, 1:3);

        % Position and orientation error
        ep = pd - p;
        Re = Rd * R';
        eo = 0.5 * [Re(3,2) - Re(2,3);
                    Re(1,3) - Re(3,1);
                    Re(2,1) - Re(1,2)];
        e = [ep; eo];

        % Stopping condition
        if norm(e) < tol
            break;
        end

        % Control law
        Ve = Kp * e;
        q_dot = pinv(J) * Ve;
        q = q + q_dot * dt;
    end

    q_out = q;
end












