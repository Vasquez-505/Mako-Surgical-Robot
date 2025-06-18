function ddq = directDynamics_numeric(q, dq, tau)

    % Compute dynamic matrices 
    M = Numeric_inertia_matrix(q);
    C = Numeric_coriolisMatrix_C(q, dq);
    G = Numeric_gravityVector_G(q);

    % Solve for joint accelerations
    ddq = M \ (tau - C * dq - G);  % More numerically stable than inv(M) * ...
end


% NOTE: This function is coppied and pasted into a Simulink Matlab function
% manually (it took to long to do it symbolic)