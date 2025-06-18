%% RIO Denavit–Hartenberg Parameters Definition Function

function Robot = RIO()

%   Returns D-H table of parameters for Robotic Arm
%   Robot=[d v a alpha offset;
%          d v a alpha offset;
%          . . .   .   offset;
%          d v a alpha offset];
%   Use symbolic variables for each joint coordinate of the robot: in the d
%   column for a prismatic joint and in the v column for a rotational
%   joint. Name the variables from q1 to qn. In the last column, insert the
%   coordinate offset for the manipulator Home position.

syms q1 q2 q3 q4 q5 q6 real
%syms d3 d5 a4 d6 d6p gama  real


d3=0.3; d5=0.5; a4=0.1; d6=0.1; d6p= 0.115; gama=pi/8;
Robot =[0    q1    0    pi/2    pi/2;
        0    q2    0   -pi/2    pi/2;
       -d3   q3    0    pi/2    pi/2;
        0    q4    a4   pi/2   -pi/2;
        d5   q5    0   -pi/2    0   ;
       -d6   q6    0    pi/2-gama -pi/2;
        d6p   0    0    0       0]; 

    

