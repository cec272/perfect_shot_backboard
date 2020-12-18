% Harrison Hidalgo
% 
%
%

function Plot_Motor_Setup(figure_number,front,up,rG,rG0,motor_rod10,motor_rod20,motor_rod30,gear2_1,gear2_2,gear2_3,thickness,spatial)

right = cross(front,up);

names = fieldnames(spatial);
for i=1:length(names)
    eval([names{i} ' = spatial.' names{i} ';']);
end
psi = spatial.psi;

rP10 = rG0-thickness/2*front-(Gx-lam1)*right-(Gy-mu1)*up;
rP20 = rG0-thickness/2*front-(Gx-lam2)*right-(Gy-mu2)*up;
rP30 = rG0-thickness/2*front-(Gx-lam3)*right-(Gy-mu3)*up;

R = [1,0,0;0,cos(psi),-sin(psi);0,sin(psi),cos(psi)]*[cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)]*[cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
rP1 = rG + R*(rP10-rG0);
rP2 = rG + R*(rP20-rG0);
rP3 = rG + R*(rP30-rG0);

%% Thetas
th2_1 = -(th1_1-th1_10) * r1/r2;
th2_2 = -(th1_2-th1_20) * r1/r2;
th2_3 = -(th1_3-th1_30) * r1/r2;

motor_rod1 = dot(motor_rod10,motor_up1)*motor_up1+cos(th2_1)*motor_rod10+sin(th2_1)*cross(motor_up1,motor_rod10);
motor_rod2 = dot(motor_rod20,motor_up2)*motor_up2+cos(th2_2)*motor_rod20+sin(th2_2)*cross(motor_up2,motor_rod20);
motor_rod3 = dot(motor_rod30,motor_up3)*motor_up3+cos(th2_3)*motor_rod30+sin(th2_3)*cross(motor_up3,motor_rod30);

% Motor points
motor_point_1 = gear2_1+motor_rod1;
motor_point_2 = gear2_2+motor_rod2;
motor_point_3 = gear2_3+motor_rod3;

%% Plotting
figure(figure_number);hold on;

% Plot gear 2s
plot3(gear2_1(1),gear2_1(2),gear2_1(3),'c.','MarkerSize',20)
plot3(gear2_2(1),gear2_2(2),gear2_2(3),'c.','MarkerSize',20)
plot3(gear2_3(1),gear2_3(2),gear2_3(3),'c.','MarkerSize',20)

% Plot motor points
plot3(motor_point_1(1),motor_point_1(2),motor_point_1(3),'y.','MarkerSize',20)
plot3(motor_point_2(1),motor_point_2(2),motor_point_2(3),'y.','MarkerSize',20)
plot3(motor_point_3(1),motor_point_3(2),motor_point_3(3),'y.','MarkerSize',20)

% Plot connections between gears and motor points
plot3([gear2_1(1),motor_point_1(1)],[gear2_1(2),motor_point_1(2)],[gear2_1(3),motor_point_1(3)],'Color','#D95319','LineWidth',2)
plot3([gear2_2(1),motor_point_2(1)],[gear2_2(2),motor_point_2(2)],[gear2_2(3),motor_point_2(3)],'Color','#D95319','LineWidth',2)
plot3([gear2_3(1),motor_point_3(1)],[gear2_3(2),motor_point_3(2)],[gear2_3(3),motor_point_3(3)],'Color','#D95319','LineWidth',2)

% Plot connections between motor points and connection points
plot3([rP1(1),motor_point_1(1)],[rP1(2),motor_point_1(2)],[rP1(3),motor_point_1(3)],'Color','#7E2F8E','LineWidth',2)
plot3([rP2(1),motor_point_2(1)],[rP2(2),motor_point_2(2)],[rP2(3),motor_point_2(3)],'Color','#7E2F8E','LineWidth',2)
plot3([rP3(1),motor_point_3(1)],[rP3(2),motor_point_3(2)],[rP3(3),motor_point_3(3)],'Color','#7E2F8E','LineWidth',2)

end








