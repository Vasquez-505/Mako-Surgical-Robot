%% Robot Dynamic Properties  

% NOTES:
% Arm length stretch no longer than 1 meter
% Robot's total mass should be no more than 30kg (mechanism intuition)

% DH Previous parameters
d3=0.3; d5=0.5; a4=0.1; d6=0.1; d6p= 0.115; gama=pi/8;
% ----------------------------------------------------

% ------------------ BODY 1: Sphere ------------------
m1 = 6.0;                % Mass (kg)  
r1 = 0.15;                % radius(m)
CoM1 = [0; 0; 0];        % CoM at center
I1 = diag ([0, (2/5)*m1*r1^2, 0]);

% ------------------ BODY 2: Sphere ------------------
m2 = 4.0;
r2 = 0.1;
CoM2 = [0; 0; 0];
I2 = (2/5)*m2*r2^2 * eye(3);

% ------------------ BODY 3: Cylinder ----------------
m3 = 3.5;
L3 = d3;
r3 = 0.05;
CoM3 = [0; L3/2; 0];     % cylinder along Z
I3 = diag([(1/12)*m3*(3*r3^2 + L3^2), ...
           (1/12)*m3*(3*r3^2 + L3^2), ...
           0.5*m3*r3^2]);

R_cg = [-1 0 0;
        0 0 -1;
        0 -1 0];

I3 = R_cg * I3 * R_cg';

% ------------------ BODY 4: Cube --------------------
m4 = 3.0;
a4 = 0.1;                % cube side length
CoM4 = [0; 0; 0];     
I4 = (1/6)*m4*a4^2 * eye(3);

% ------------------ BODY 5: Cylinder ----------------
m5 = 3.0;
L5 = d5;
r5 = 0.03;
CoM5 = [0; L5/2; 0];
I5 = diag([(1/12)*m5*(3*r5^2 + L5^2), ...
           (1/12)*m5*(3*r5^2 + L5^2), ...
           0.5*m5*r5^2]);

% ------------------ BODY 6: Cylinder ----------------
m6 = 1.0;
L6 = 0.115;
r6 = 0.015;
CoM6 = [0; 0; -L6/2];
I6 = diag([(1/12)*m6*(3*r6^2 + L6^2), ...
           (1/12)*m6*(3*r6^2 + L6^2), ...
           0.5*m6*r6^2]);

I6 = R_cg * I6 * R_cg';

% ------------------ Create Struct -------------------
robot(1).mass = m1; robot(1).CoM = CoM1; robot(1).inertia = I1;
robot(2).mass = m2; robot(2).CoM = CoM2; robot(2).inertia = I2;
robot(3).mass = m3; robot(3).CoM = CoM3; robot(3).inertia = I3;
robot(4).mass = m4; robot(4).CoM = CoM4; robot(4).inertia = I4;
robot(5).mass = m5; robot(5).CoM = CoM5; robot(5).inertia = I5;
robot(6).mass = m6; robot(6).CoM = CoM6; robot(6).inertia = I6;

% Pack robot fields into separate arrays for numeric calls
masses = [robot.mass];
CoMs = reshape([robot.CoM], 3, []);
inertias = reshape([robot.inertia], 3, 3, []);

save('mako_robot_mass_properties.mat', 'robot', 'masses', 'CoMs', 'inertias');
disp("Mass properties saved to mako_robot_mass_properties.mat");

total_mass = sum(masses);
disp(['Total robot mass: ', num2str(total_mass), ' kg']);
