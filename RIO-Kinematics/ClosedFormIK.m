%% Closed Form Inverse Kinematics Function

function q = ClosedFormIK(pd, Rd)
% ClosedFormIK - Computes inverse kinematics for RIO robot using closed-form method.
% Inputs:
%   pd : 3x1 desired end-effector position [x; y; z]
%   Rd : 3x3 desired end-effector orientation matrix
% Output:
%   q  : 6x1 joint angles [q1; q2; q3; q4; q5; q6]

    % Robot constants (match the RIO DH parameters)
    d3 = 0.3;
    d5 = 0.5;
    a4 = 0.1;
    d6 = 0.1;
    d6p = 0.115;
    gama = pi/8;

    % Adjust for tool offset
    R06 = Rd;
    p06 = pd - R06 * [0; 0; d6p];

    % Wrist center position
    pw = p06 - R06 * [0; 0; d6];

    % Extract wrist center coordinates
    x = pw(1);
    y = pw(2);
    z = pw(3);

    % Solve for q1
    q1 = atan2(y, x) - pi/2;

    % Solve for q2 and q3 using geometric IK
    r = sqrt(x^2 + y^2);
    z3 = z + d3;

    % Law of cosines term D
    D = (r^2 + z3^2 - a4^2 - d5^2) / (2 * a4 * d5);

    % Debug print
    fprintf('Computed D = %.6f\n', D);

    % Check if position is reachable within a tolerance
    tol_reach = 1e-6;
    if D > 1 + tol_reach || D < -1 - tol_reach
        error('Position unreachable. No valid IK solution.');
    end

    % Clamp D if it is slightly out of range due to numerical errors
    D = min(max(D, -1), 1);

    % Choose elbow-down solution
    q3 = atan2(sqrt(1 - D^2), D);

    phi = atan2(z3, r);
    psi = atan2(d5 * sin(q3), a4 + d5 * cos(q3));
    q2 = - (phi + psi) - pi/2;

    % Forward kinematics to get R03 rotation matrix
    R03 = rotz(q1 + pi/2) * roty(q2 + pi/2) * rotx(q3);

    % Normalize R03 just in case
    [U,~,V] = svd(R03);
    R03 = U*V';

    % Rotation from joint 3 to end-effector
    R36 = R03' * R06;

    % Solve for wrist joints q4, q5, q6 from R36
    q4 = atan2(R36(2,3), R36(1,3));
    q5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
    q6 = atan2(R36(3,2), -R36(3,1));

    % Final joint vector
    q = [q1; q2; q3; q4; q5; q6];
end


% Rotation matrices auxiliary functions
function Rz = rotz(theta)
    c = cos(theta); s = sin(theta);
    Rz = [c -s 0;
          s  c 0;
          0  0 1];
end

function Ry = roty(theta)
    c = cos(theta); s = sin(theta);
    Ry = [ c 0 s;
           0 1 0;
          -s 0 c];
end

function Rx = rotx(theta)
    c = cos(theta); s = sin(theta);
    Rx = [1  0  0;
          0  c -s;
          0  s  c];
end







