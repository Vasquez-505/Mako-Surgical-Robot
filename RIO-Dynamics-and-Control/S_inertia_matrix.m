function M = S_inertia_matrix(q)
    
    % Load mass and geometry
    load('mako_robot_mass_properties.mat', 'robot');
    DH = RIO();  % Symbolic DH table 

    n = 6; % Number of joints 
    M = sym(zeros(n));

    % Initial transform
    T = eye(4);
    o = sym(zeros(3, n+1));
    z = sym(zeros(3, n+1));
    o(:,1) = [0;0;0];
    z(:,1) = [0;0;1];

    for i = 1:n
        A = DHTransf(DH(i,:));
        T = T * A;
        o(:,i+1) = T(1:3,4);
        z(:,i+1) = T(1:3,3);

        % CoM in base frame
        r_c = T(1:3,1:3) * robot(i).CoM + T(1:3,4);

        % Build Jacobians to CoM
        Jv = sym(zeros(3,n));
        Jw = sym(zeros(3,n));
        for j = 1:i
            if isrevolute(DH(j,:))
                Jv(:,j) = cross(z(:,j), r_c - o(:,j));
                Jw(:,j) = z(:,j);
            else
                Jv(:,j) = z(:,j);
                Jw(:,j) = sym([0;0;0]);
            end
        end

        % Inertia matrix for link i
        m = robot(i).mass;
        I = robot(i).inertia;
        R = T(1:3,1:3);
        I_base = R * I * R';

        M = M + (m * Jv.' * Jv + Jw.' * I_base * Jw);
    end

    % Using 'Steps' to limit simplification time and complexity
    M = simplify(M, 'Steps', 30);
end

% Helper Function: Checks DH column joint type
function flag = isrevolute(dh_row)
% Returns true if the joint is revolute, based on symbolic
% presence in column 2 (theta)
    flag = ~isempty(symvar(dh_row(2)));
end