% Harrison Hidalgo
% ECE 5725 - Final Project
%
%

function x_dot = ball_calc(~,x,p)
%% Unpack X
X=x(1);Y=x(2);Z=x(3);X_dot=x(4);Y_dot=x(5);Z_dot=x(6);
%% Unpack p
c=p.c;g=p.g;m=p.m;
%% Calculations 
Mass = A_ball(m);
Force = b_ball(c,g,m,X_dot,Y_dot,Z_dot);
%% Pack up solution
x_dot = [X_dot;Y_dot;Z_dot;Mass\Force];
end