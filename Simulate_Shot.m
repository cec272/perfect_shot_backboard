% Harrison Hidalgo 
% ECE 5725 - Final Project
% Ball flighing through air
%
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
W_of_backboard = 1.5;
H_of_backboard = 0.5;
T_of_backboard = 0.1;

front = [1,0,0]';
up = [0,1,0]';
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
%% Initialize ball
ball = plot3(X_pos,Y_pos,Z_pos,'.','MarkerSize',50);
xlabel('X');ylabel('Y');zlabel('Z');
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
