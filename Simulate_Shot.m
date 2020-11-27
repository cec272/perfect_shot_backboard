% Harrison Hidalgo 
% ECE 5725 - Final Project
% Ball flying through air
%

% Color key:
% red    = backboard
% blue   = points connected to motors
% black  = backboard CoM
% green  = point that can only move along x
% cyan   = Gear number 2
% yellow = motor points
% orange = lever/motor rod
% purple = socket rod

clc;clear;close all;
%% Position Constants
X_of_backboard = 0;
Y_of_backboard = 0;
Z_of_backboard = 10;
Z_dif_of_hoop  = -1.2;
X_dif_of_hoop  = 0.4;
r_of_hoop      = 0.25;

%%
backboard = [X_of_backboard;Y_of_backboard;Z_of_backboard];
center_hoop = backboard + [X_dif_of_hoop;0;Z_dif_of_hoop];
%% Sizes
r_of_ball      = 0.125;
W_of_backboard = 0.75;
H_of_backboard = 0.5;
T_of_backboard = 0.1;

front = [1,0,0]';
up = [0,0,1]';
%%
%% Initial States
% Positions
X_pos = 2;
Y_pos = 2;
Z_pos = 6;
% Velocities
X_vel = -2;
Y_vel = -2;
Z_vel = 15;

%% Gear position
gear2_1 = [-0.3;0.3;10];
gear2_2 = [-0.3;0;10];
gear2_3 = [-0.3;-0.3;10];

%% Motor rod positions
motor_rod10 = [0.05;0;0];
motor_rod20 = [0.05;0;0];
motor_rod30 = [0.05;0;0];

%% Physical variables
air_drag_coefficient = 0.1;
acceleration_of_gravity = 10;
mass_of_ball = 1;
coefficient_of_restitution = 1;
%%
x = [X_pos;Y_pos;Z_pos;X_vel;Y_vel;Z_vel];
p.c=air_drag_coefficient;p.g=acceleration_of_gravity;p.m=mass_of_ball;
p.e=coefficient_of_restitution;
%% Initialize surroundings
Plot_Box(backboard,front,up,W_of_backboard,H_of_backboard,T_of_backboard,1)
n=100;angle=linspace(0,2*pi,n);
plot3(r_of_hoop*cos(angle)+center_hoop(1),r_of_hoop*sin(angle)+center_hoop(2),center_hoop(3)*ones(n,1));
%%
spatial.Gx = W_of_backboard/2;
spatial.Gy = H_of_backboard/2;
spatial.lam1 = W_of_backboard/4;
spatial.lam2 = W_of_backboard/2;
spatial.lam3 = W_of_backboard*3/4;
spatial.mu1 = H_of_backboard*2/3;
spatial.mu2 = H_of_backboard/3;
spatial.mu3 = H_of_backboard*2/3;

spatial.beta1 = W_of_backboard/2;
spatial.alpha1 = H_of_backboard/2;

spatial.th1_1 = 0;
spatial.th1_2 = 0;
spatial.th1_3 = 0;

spatial.th1_10 = pi/2;
spatial.th1_20 = -pi/2;
spatial.th1_30 = -pi/2;

spatial.r1 = 0.1;
spatial.r2 = 0.1;

spatial.motor_up1 = up;
spatial.motor_up2 = up;
spatial.motor_up3 = up;

spatial.phi = 0;
spatial.psi = 0;
spatial.theta = 0;
G0=backboard;
G=G0;
%%
Plot_Relevant_Points(G0,G,front,up,spatial,T_of_backboard,1)
Plot_Motor_Setup(1,front,up,G,G0,motor_rod10,motor_rod20,motor_rod30,gear2_1,gear2_2,gear2_3,T_of_backboard,spatial)
return
%% Initialize ball
ball = plot3(X_pos,Y_pos,Z_pos,'.','MarkerSize',50);
start_time=0;h=0.05;TFinal = 7;
ffun="ball_calc";
t_array=start_time;x_array = x;collision_tracker=0;
for i = 1:TFinal/h
    [t_new,x_new] = runge_kutta(h,t_array(i),x_array(:,i),ffun,p);
    ball.XData = x_new(1);
    ball.YData = x_new(2);
    ball.ZData = x_new(3);
    if (collision_tracker < 1) && did_it_collide(backboard,x_new(1:3),front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball)
        x_new(4:6) = collision(up,front,x_new(4:6),p.e);
        disp('yes')
        collision_tracker=collision_tracker+1;
    elseif (collision_tracker < 5)&&(collision_tracker>0)
        collision_tracker=collision_tracker+1;
    elseif (collision_tracker > 5)
        collision_tracker = 0;
    end
    t_array = [t_array,t_new];x_array = [x_array,x_new];
    pause(0.1)
end


figure(2);hold on
plot3(x_array(1,:),x_array(2,:),x_array(3,:),'LineWidth',3)
Plot_Box(backboard,front,up,W_of_backboard,H_of_backboard,T_of_backboard,2)
n=100;angle=linspace(0,2*pi,n);
plot3(r_of_hoop*cos(angle)+center_hoop(1),r_of_hoop*sin(angle)+center_hoop(2),center_hoop(3)*ones(n,1));
