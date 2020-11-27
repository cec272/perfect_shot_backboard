% Harrison Hidalgo
% ECE 5725 - Final Project
%
%

%   Back |-----------------------------------|
%        |                                   |
% -------|----------------------.            |
% Gy     |                      |G           |
% |  ----|-------------.        |            |
% |alpha1|             |B1      |            |
% | | ---|----.        |        |            |
% | | mu1|    |P1      |        |            |
% | |  | |    |        |        |            |
% -------|-----------------------------------|
%        |lam1|        |        |
%        |-----beta1---|        |
%        |--------Gx------------|
%   Key:
%       P = Attachment point from motors
%       B = Constraint attachment
%       G = Center of mass of backboard
%

%   theta = rotation about z axis
%   phi   = rotation about y axis
%   psi   = rotation about x axis

clc;clear;close all;
fprintf('Starting calculation\n')
%% Initialize symbols
syms I1 I2 I3 m g real % Moment of inertia and mass of backboard, gravity
syms T1 T2 T3 real % Input torques
syms Gx Gy beta1 alpha1 lam1 lam2 lam3 mu1 mu2 mu3 real % Positions on backboard (see above)
syms r1 r2 eta real % radius of 1st and 2nd gears, efficiency between gears
%% Rotation and position of backboard

syms theta phi psi real % rotation about Z, Y, X
syms theta_dot phi_dot psi_dot real 
syms theta_ddot phi_ddot psi_ddot real
syms x y z real % CoM of backboard
syms x_dot y_dot z_dot real
syms x_ddot y_ddot z_ddot real

syms front1 front2 front3 real % Unit vector orthogonal to front of backboard pointing out
syms up1 up2 up3 real % Unit vector orthogonal to top of backboard
syms Gx0 Gy0 Gz0 real % Initial position of CoM of backboard
syms thickness real % Thickness of backboard

%% Motors
syms th1_1 th1_2 th1_3 real % Orientation of motors
syms th1_10 th1_20 th1_30 real % Initial orientation of motors

%% Original direction of motor rods
syms motor_rod101 motor_rod201 motor_rod301 real
syms motor_rod102 motor_rod202 motor_rod302 real
syms motor_rod103 motor_rod203 motor_rod303 real
% Put orginal orientations of motor rods into unit vectors
motor_rod10 = [motor_rod101;motor_rod102;motor_rod103];
motor_rod20 = [motor_rod201;motor_rod202;motor_rod203];
motor_rod30 = [motor_rod301;motor_rod302;motor_rod303];
%% Unit vectors on rotation axis of motors
syms motor_up11 motor_up21 motor_up31 real
syms motor_up12 motor_up22 motor_up32 real
syms motor_up13 motor_up23 motor_up33 real
%% Motor unit vectors
motor_up1=[motor_up11;motor_up12;motor_up13];
motor_up2=[motor_up21;motor_up22;motor_up23];
motor_up3=[motor_up31;motor_up32;motor_up33];
%% Positions of second gears
syms gear2_11 gear2_21 gear2_31 real
syms gear2_12 gear2_22 gear2_32 real
syms gear2_13 gear2_23 gear2_33 real
% Put into vectors
gear2_1 = [gear2_11;gear2_12;gear2_13];
gear2_2 = [gear2_21;gear2_22;gear2_23];
gear2_3 = [gear2_31;gear2_32;gear2_33];
%% Forces at point B
syms Fy Fz real
fprintf('Symbols have been declared\n')
%% Creation of I matrix
I0 = diag([I1,I2,I3]);
R = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1]*[cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)]*[1,0,0;0,cos(psi),-sin(psi);0,sin(psi),cos(psi)];
I = R*I0*R';
%% Create unit vectors
i=[1;0;0];j=[0;1;0];k=cross(i,j);
front=[front1;front2;front3];up=[up1;up2;up3];right=cross(front,up);
fprintf('Positions\n')
%% Absolute Positions
% Original positions
rG0  = Gx0*i + Gy0*j + Gz0*k;
rP10 = rG0-thickness/2*front-(Gx-lam1)*right-(Gy-mu1)*up;
rP20 = rG0-thickness/2*front-(Gx-lam2)*right-(Gy-mu2)*up;
rP30 = rG0-thickness/2*front-(Gx-lam3)*right-(Gy-mu3)*up;
rB0  = rG0-thickness/2*front-(Gx-beta1)*right-(Gy-alpha1)*up;
% New Positions
rG = x*i + y*j + k*z;
rP1 = rG + R*(rP10-rG0);
rP2 = rG + R*(rP20-rG0);
rP3 = rG + R*(rP30-rG0);
rB  = rG + R*(rB0-rG0);
%% Motor Stuff
% Calculate Thetas
th2_1 = -(th1_1-th1_10) * r1/r2;
th2_2 = -(th1_2-th1_20) * r1/r2;
th2_3 = -(th1_3-th1_30) * r1/r2;

% Rotation about an arbitrary axis
motor_rod1 = dot(motor_rod10,motor_up1)*motor_up1+cos(th2_1)*motor_rod10+sin(th2_1)*cross(motor_up1,motor_rod10);
motor_rod2 = dot(motor_rod20,motor_up2)*motor_up2+cos(th2_2)*motor_rod20+sin(th2_2)*cross(motor_up2,motor_rod20);
motor_rod3 = dot(motor_rod30,motor_up3)*motor_up3+cos(th2_3)*motor_rod30+sin(th2_3)*cross(motor_up3,motor_rod30);

% Motor points
motor_point_1 = gear2_1+motor_rod1;
motor_point_2 = gear2_2+motor_rod2;
motor_point_3 = gear2_3+motor_rod3;

%% Relative Positions
% Motor points to backboard points
r_MP1P1 = rP1-motor_point_1;
r_MP2P2 = rP2-motor_point_2;
r_MP3P3 = rP3-motor_point_3;

% Center of mass to backboard points
rGP1 = rP1-rG;
rGP2 = rP2-rG;
rGP3 = rP3-rG;
rGB  = rB -rG;
fprintf('Dynamics\n')
%% Forces
FP1 = (-T1*eta/r2)*r_MP1P1/norm(r_MP1P1); % Magnitude from torque
FP2 = (-T2*eta/r2)*r_MP2P2/norm(r_MP2P2); % Direction from constraint
FP3 = (-T3*eta/r2)*r_MP3P3/norm(r_MP3P3);
FB  = Fy*j + Fz*k; % Free to move along x
FG  = -m*g*j;% Gravity

%% Moments
M_FP1 = cross(rGP1,FP1); % M = r x F
M_FP2 = cross(rGP2,FP2);
M_FP3 = cross(rGP3,FP3);
M_FB  = cross(rGB,FB);

%% Change in Angular Momentum
omega   = [theta_dot;phi_dot;psi_dot];
omega_d = [theta_ddot;phi_ddot;psi_ddot];
H_dot = I*omega_d + cross(omega,I*omega); % Fancy rate of change of angular momentum

%% Change in Linear Momentum
L_dot = m*(x_ddot*i+y_ddot*j+z_ddot*k);

%% Inertial Balances
eqns1 = L_dot - FP1 - FP2 - FP3 - FB - FG;
eqns2 = H_dot - M_FP1 - M_FP2 - M_FP3 - M_FB;

%% Constraints
vars    = [x;y;z;theta;phi;psi;Fy;Fz];
vars_d  = [x_dot;y_dot;z_dot;theta_dot;phi_dot;psi_dot;Fy;Fz];
vars_dd = [x_ddot;y_ddot;z_ddot;theta_ddot;phi_ddot;psi_ddot;Fy;Fz];
B_d = jacobian(rB,vars)*vars_d;
B_dd = jacobian(B_d,vars_d)*vars_dd+jacobian(B_d,vars)*vars_d;

%% Calculate Mass and Force tensors
fprintf('Calculating EoMs\n')
equations=[eqns1(1);eqns1(2);eqns1(3);eqns2(1);eqns2(2);eqns2(3);B_dd(1);B_dd(2)];
M = (jacobian(equations,vars_dd))
b = (M*vars_dd - equations)

%% Packaging EoMs
fprintf('Packing equations up\n')
matlabFunction(M,'File','A_backboard');
matlabFunction(b,'File','b_backboard');
