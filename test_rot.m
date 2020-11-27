

clear;close all; clear;
I1=1;I2=1;I3=1;
p.I = diag([I1,I2,I3]);
p.M = [1;2;3];

theta = 0;
phi = 0; 
psi = 0;

theta_dot = 0;
phi_dot   = 0;
psi_dot   = 0;

inits=[theta;phi;psi;theta_dot;phi_dot;psi_dot];

N = 1000;
tspan=linspace(0,10,N);

f = @(t,x) rhs(t,x,p);

opts.RelTol = 1e-6;
opts.AbsTol = 1e-6;

[~,xarray] = ode45(f,tspan,inits,opts);
r1 = [1;0;0];
r2 = [0;1;0];
r3 = [0;0;1];
figure(1);hold on;
a=plot3(r1(1),r1(2),r1(3),'r.','MarkerSize',50);
b=plot3(r2(1),r2(2),r2(3),'b.','MarkerSize',50);
c=plot3(r3(1),r3(2),r3(3),'g.','MarkerSize',50);
    view([60,30])
    xlim([-1.5,1.5]);ylim([-1.5,1.5]);zlim([-1.5,1.5]);
    xlabel('X');ylabel('Y');zlabel('Z');
    r=[];
for i = 1:N
    theta = xarray(i,1);
    phi = xarray(i,2);
    psi = xarray(i,3);
    Rx = [1,0,0;0,cos(psi),-sin(psi);0,sin(psi),cos(psi)];
    Ry = [cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)];
    Rz = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
    R = Rz*Ry*Rx;
    r_new_1 = R*r1;
    r_new_2 = R*r2;
    r_new_3 = R*r3;
    a.XData = r_new_1(1);
    a.YData = r_new_1(2);
    a.ZData = r_new_1(3);
    
    b.XData = r_new_2(1);
    b.YData = r_new_2(2);
    b.ZData = r_new_2(3);
    
    c.XData = r_new_3(1);
    c.YData = r_new_3(2);
    c.ZData = r_new_3(3);
    
    r = [r,r_new_1];
    
    
    pause(0.01)
end

figure(2); hold on;
plot(tspan,xarray(:,1));
plot(tspan,xarray(:,2));
plot(tspan,xarray(:,3));

figure(3); hold on;
plot(tspan,r(1,:));
plot(tspan,r(2,:));
plot(tspan,r(3,:));

function x_dot = rhs(t,x,p)

omega =[x(4);x(5);x(6)];
theta_dot = omega;
omega_dot = inv(p.I)*(p.M-cross(omega,p.I*omega));
x_dot = [theta_dot;omega_dot];
end