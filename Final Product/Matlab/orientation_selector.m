% Harrison Hidalgo
% ECE 5725 - Final Project
% Selects orientation to move to using mean-square error.
%

function [theta1,theta2,theta3] = orientation_selector(th_1_current,th_2_current,th_3_current,possible_orientations)
min_MSE = inf;
for i = 1:length(possible_orientations)
    MSE_th1 = (th_1_current-possible_orientation(1))^2;
    MSE_th2 = (th_2_current-possible_orientation(2))^2;
    MSE_th3 = (th_3_current-possible_orientation(3))^2;
    if ((MSE_th1+MSE_th2+MSE_th3)/3) < min_MSE
        theta1=possible_orientation(1);
        theta2=possible_orientation(2);
        theta3=possible_orientation(3);    
    end
end
end