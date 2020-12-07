% Harrison Hidalgo
% ECE 5725 - Final Project
% This function will simulate a collision.
%

function [velocity_new] = collision(up,front,velocity,restitution)
%% Calculate the left component of the backboard.
left = cross(front,up);
%% Decompose velocity into backboard coordinates.
up_comp = dot(velocity,up)/dot(up,up)*up;
front_comp = dot(velocity,front)/dot(front,front)*front;
left_comp = dot(velocity,left)/dot(left,left)*left;
%% Put velocity back together.
velocity_new = up_comp-front_comp*restitution+left_comp;
end