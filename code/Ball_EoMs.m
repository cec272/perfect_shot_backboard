% Harrison Hidalgo
% ECE 5725 - Final Project
% Symbolic toolbox to find EoMs of ball
%

%% Declare symbols
syms x_vel y_vel z_vel x_acc y_acc z_acc real
syms m g c real

%% Unit vectors
i = [1;0;0];j = [0;1;0]; k = cross(i,j);

%% Velocity
velocity = [x_vel;y_vel;z_vel];

%% Another Unit Vector
unit_velocity = velocity/norm(velocity);

%% Calcualte Forces
F_drag = c*norm(velocity)^2*(-unit_velocity);
F_gravity = m*g*(-k);

%% Linear Momentum Balance
sum_F = F_drag+F_gravity;
L_dot = m*[x_acc;y_acc;z_acc];
eqns = L_dot-sum_F;

%% Create mass and force vectors
M = simplify(jacobian(eqns,[x_acc,y_acc,z_acc]));
b = simplify(M*[x_acc;y_acc;z_acc] - eqns);

%% Package mass and force vectors
matlabFunction(M,'File','A_ball');
matlabFunction(b,'File','b_ball');

