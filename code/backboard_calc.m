% Harrison Hidalgo
% ECE 5725 - Final Project
% Right Hand Side equation for backboard runge-kutta
%

function x_dot=backboard_calc(~,X,p)
% Unpack p
names = fieldnames(p);
for i=1:length(names)
    eval([names{i} ' = p.' names{i} ';']);
end

% Unpack x
x=X(1);y=X(2);z=X(3);theta=X(4);phi=X(5);psi=X(6);
x_dot=X(7);y_dot=X(8);z_dot=X(9);
theta_dot=X(10);phi_dot=X(11);psi_dot=X(12);

% Calculate X_dot
M = A_backboard(Gx,Gy,I1,I2,I3,alpha1,beta1,front1,front2,front3,m,phi,...
    psi,theta,thickness,up1,up2,up3);

b = b_backboard(Gx,Gy,I1,I2,I3,T1,T2,T3,alpha1,beta1,eta,front1,front2,...
    front3,g,gear2_11,gear2_12,gear2_13,gear2_21,gear2_22,gear2_23,...
    gear2_31,gear2_32,gear2_33,lam1,lam2,lam3,m,motor_up11,motor_up12,...
    motor_up13,motor_up21,motor_up22,motor_up23,motor_up31,motor_up32,...
    motor_up33,motor_rod101,motor_rod102,motor_rod103,motor_rod201,...
    motor_rod202,motor_rod203,motor_rod301,motor_rod302,motor_rod303,...
    mu1,mu2,mu3,phi,phi_dot,psi,psi_dot,r1,r2,th1_1,th1_2,th1_3,th1_10,...
    th1_20,th1_30,theta,theta_dot,thickness,up1,up2,up3,x,y,z);

% Pack up x_dot
x_dot = [x_dot;y_dot;z_dot;theta_dot;phi_dot;psi_dot;M\b];
end