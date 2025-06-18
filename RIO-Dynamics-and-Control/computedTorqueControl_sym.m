function tau_sym = computedTorqueControl_sym()
    % Define symbolic variables
    syms q [6 1] real
    syms dq [6 1] real
    syms qd [6 1] real
    syms dqd [6 1] real
    syms ddqd [6 1] real
    syms kp [6 1] real
    syms kd [6 1] real

    % Define symbolic Kp and Kd diagonal matrices
    Kp = diag(kp);
    Kd = diag(kd);

    % Load or define symbolic dynamics
    [M_sym, C_sym, G_sym] = compute_dynamics(q, dq);  

    % Compute error
    e = qd - q;
    de = dqd - dq;

    % Compute symbolic torque
    tau_sym = M_sym * (ddqd + Kd * de + Kp * e) + C_sym * dq + G_sym;
end