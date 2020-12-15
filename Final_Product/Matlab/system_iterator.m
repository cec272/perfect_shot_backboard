% Harrison Hidalgo
% ECE 5725 - Final Project
% This function will cycle through possible orientations of the backboard.
% It will output the possible orientations that would be solutions to the
% system, i.e. make a basket.

function working_orientations = system_iterator(iteration_vals,e_b,r_B0,h,x_ball_init,ffun,p,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop)
% Unpack iteration_vals
names = fieldnames(iteration_vals);
for i=1:length(names)
    eval([names{i} ' = iteration_vals.' names{i} ';']);
end
% Can cycle through different theta, psi, phi, and B positions along e_b
number_of_divisions = 10; % Keep in mind the number of iterations is N^4
% Create vectors to iterate through
theta_iter = linspace(theta_min,theta_max,number_of_divisions);
phi_iter = linspace(phi_min,phi_max,number_of_divisions);
psi_iter = linspace(psi_min,psi_max,number_of_divisions);
B_iter = linspace(B_min,B_max,number_of_divisions);
% For storing results
working_orientations = [];

for i=1:number_of_divisions% theta
for j=1:number_of_divisions% phi
for k=1:number_of_divisions% psi
for l=1:number_of_divisions% B
    theta=theta_iter(i);phi=phi_iter(j);psi=psi_iter(k);r_B=e_b*B_iter(l)+r_B0;
    R = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1]*[cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)]*[1,0,0;0,cos(psi),-sin(psi);0,sin(psi),cos(psi)];
    new_up = R*up;new_front = R*front;
    new_backboard = r_B-R*rGB0;
    if path_tracker(h,x_ball_init,ffun,p,new_front,new_up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop,new_backboard)
        working_orientations = [possible combo,[theta_iter(i);phi_iter(j);psi_iter(k);B_iter(l)]];
    end
end 
end
end
end
end