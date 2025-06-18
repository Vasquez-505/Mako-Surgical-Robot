%% Direct Kinematics (& Jacobian Computation) Function 

function [T, J] = DKin(Robot)

    % Computes:
    % T - Homogeneous Transformation from base to end-effector
    % J - 6xN Geometric Jacobian 

    n = size(Robot,1) - 1;     % Last row is tool transform (not a joint)
    T = eye(4);
    J = sym(zeros(6,n));       % Initialize Jacobian

   what  % Store origins and z-axes for each frame
    o = sym(zeros(3,n+1));     % Origins (including base)
    z = sym(zeros(3,n+1));     % Z-axes (including base)

    o(:,1) = [0; 0; 0];         % Base frame origin
    z(:,1) = [0; 0; 1];         % Base frame Z-axis

    % Build transformation and store intermediate frames
    Ti = eye(4);  % Cumulative transformation
    for i = 1:n
        A = DHTransf(Robot(i,:));
        Ti = Ti * A;            
        o(:,i+1) = Ti(1:3,4);   % origin of i-th frame
        z(:,i+1) = Ti(1:3,3);   % z axis of i-th frame
    end

    % Final transformation (including tool)
    T = Ti * DHTransf(Robot(n+1,:));
    T = simplify(T);

    % End-effector position vector (used for Jacobian)
    pe = T(1:3,4);   % end-effector position in base frame

    % Build Jacobian
    for i = 1:n
        zi = z(:,i);        % z_{i-1}
        oi = o(:,i);        % o_{i-1}

        % Inline isrotational logic: check if v-column is symbolic
        is_rotational = ~isempty(symvar(Robot(i,2)));

        if is_rotational
            Jv = cross(zi, pe - oi);  % Revolute
            Jw = zi;
        else
            Jv = zi;                 % Prismatic
            Jw = sym([0; 0; 0]);
        end

        J(:,i) = [Jv; Jw];
    end

    J = simplify(J);
end


