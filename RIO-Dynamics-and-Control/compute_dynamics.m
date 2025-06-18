function [M, C, G] = compute_dynamics(q, dq)
    % Inputs:
    % q  - symbolic vector of joint positions [6x1]
    % dq - symbolic vector of joint velocities [6x1]

    % Compute Mass matrix M(q)
    M = S_inertia_matrix(q);

    % Number of joints
    n = length(q);

    % Initialize Christoffel symbols and Coriolis matrix
    C = sym(zeros(n));
    c = sym(zeros(n,n,n)); % 3D tensor for Christoffel symbols

    % Calculate Christoffel symbols c_{ijk}
    for i = 1:n
        for j = 1:n
            for k = 1:n
                c(i,j,k) = 0.5 * (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i)));
            end
        end
    end

    % Calculate Coriolis matrix C_ij = sum_k c_ijk * dq_k
    for i = 1:n
        for j = 1:n
            C(i,j) = sum(squeeze(c(i,j,:)) .* dq);
        end
    end
    
    C = simplify(C);
    
    % Compute gravity vector G(q)
    % Define gravitational acceleration vector (z0 axis has the same
    % direction as gravity)
    g_vect = [0; 0; 9.81];

    % Load robot structure for mass and CoM positions
    load('mako_robot_mass_properties.mat', 'robot');
    DH = RIO(); % symbolic DH parameters

    % Initialize potential energy
    V = sym(0);

    % Compute potential energy by summing m_i * g^T * p_ci
    T = eye(4);
    for i = 1:n
        A = DHTransf(DH(i,:));
        T = T * A;

        % CoM position of link i in base frame
        p_ci = T(1:3,1:3) * robot(i).CoM + T(1:3,4);

        % Potential energy contribution from link i
        V = V + robot(i).mass * (g_vect.' * p_ci);
    end

    % Gravity vector is the gradient of potential energy wrt q
    G = gradient(V, q);
    G = simplify(G);
end
