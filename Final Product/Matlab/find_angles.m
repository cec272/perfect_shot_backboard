% Harrison Hidalgo
% ECE 5725 - Final Project
%
%

function [does_it_work,theta_1,theta_2,theta_3] = find_angles(theta,phi,psi,r_B)
N=10;
th1=linspace(th1_start,th1_end,N);
th2=linspace(th2_start,th2_end,N);
th3=linspace(th3_start,th3_end,N);

R = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1]*[cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)]*[1,0,0;0,cos(psi),-sin(psi);0,sin(psi),cos(psi)];
rP1= r_B-R*(rGB0+rGP10);
rP2= r_B-R*(rGB0+rGP20);
rP3= r_B-R*(rGB0+rGP30);

does_it_work=0;
for i = 1:N
    for j = 1:N
        for k = 1:N
th2_1=th1(i);th2_2=th2(j);th2_3=th3(k);
% Rotation about an arbitrary axis
motor_rod1 = dot(motor_rod10,motor_up1)*motor_up1+cos(th2_1)*motor_rod10+sin(th2_1)*cross(motor_up1,motor_rod10);
motor_rod2 = dot(motor_rod20,motor_up2)*motor_up2+cos(th2_2)*motor_rod20+sin(th2_2)*cross(motor_up2,motor_rod20);
motor_rod3 = dot(motor_rod30,motor_up3)*motor_up3+cos(th2_3)*motor_rod30+sin(th2_3)*cross(motor_up3,motor_rod30);

% Motor points
motor_point_1 = gear2_1+motor_rod1;
motor_point_2 = gear2_2+motor_rod2;
motor_point_3 = gear2_3+motor_rod3;

r_P1_MP1 = motor_point_1 - rP1;
r_P2_MP2 = motor_point_2 - rP2;
r_P3_MP3 = motor_point_3 - rP3;

if (norm(r_P1_MP1-P1)-L1 < 0.1)&&(norm(r_P2_MP2-P2)-L2 < 0.1)&&(norm(r_P3_MP3-P3)-L3 < 0.1)
    does_it_work=1;theta_1 = [theta_1;th_1];theta_2=[theta_2;th_2];theta_3=[theta_3;th_3];
end
        end
    end
end
end