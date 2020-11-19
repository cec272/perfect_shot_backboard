% Harrison Hidalgo
% ECE 5275 - Final Project
%
% Front:
% A o-----------------------------o C
%   |                             |
%   |                             |
%   |                             |
%   |                             |
%   |                             |
%   |                             |
%   |                             |
% B o-----------------------------o D
%
% Back:
% E o-----------------------------o G
%   |                             |
%   |                             |
%   |                             |
%   |                             |
%   |                             |
%   |                             |
%   |                             |
% F o-----------------------------o H

function Plot_Box(G,front,up,width,height,thickness,figure_number)
    %% Calculate 3rd component of backboard coordinate system. 
    left = cross(front,up);
    %% Setting these reduces later command complexity.
    t2 = thickness/2;
    w2 = width/2;
    h2 = height/2;
    %% Calculate points from backboard coordinate system.
    pointA =  front*t2+left*w2+up*h2+G;
    pointB =  front*t2+left*w2-up*h2+G;
    pointC =  front*t2-left*w2+up*h2+G;
    pointD =  front*t2-left*w2-up*h2+G;
    pointE = -front*t2+left*w2+up*h2+G;
    pointF = -front*t2+left*w2-up*h2+G;
    pointG = -front*t2-left*w2+up*h2+G;
    pointH = -front*t2-left*w2-up*h2+G;
    %% Plotting each point.
    figure(figure_number);hold on;
    plot3(pointA(1),pointA(2),pointA(3),'r.','MarkerSize',20)
    plot3(pointB(1),pointB(2),pointB(3),'r.','MarkerSize',20)
    plot3(pointC(1),pointC(2),pointC(3),'r.','MarkerSize',20)
    plot3(pointD(1),pointD(2),pointD(3),'r.','MarkerSize',20)
    plot3(pointE(1),pointE(2),pointE(3),'r.','MarkerSize',20)
    plot3(pointF(1),pointF(2),pointF(3),'r.','MarkerSize',20)
    plot3(pointG(1),pointG(2),pointG(3),'r.','MarkerSize',20)
    plot3(pointH(1),pointH(2),pointH(3),'r.','MarkerSize',20)
    %% These commands make viewing the hoop easier.
    set(gca, 'CameraPosition', [10 10 10]);
    view([60,30])
    xlim([-1,1]);ylim([-1,1]);zlim([5,15]);
end