% Harrison Hidalgo
% ECE 5725 - Final Project
%
%

function Plot_Relevant_Points(G0,G,front,up,spatial,thickness,figure_number)

names = fieldnames(spatial);
for i=1:length(names)
    eval([names{i} ' = spatial.' names{i} ';']);
end
psi = spatial.psi;

right=cross(front,up);

P10 = G0-thickness/2*front-(Gx-lam1)*right-(Gy-mu1)*up;
P20 = G0-thickness/2*front-(Gx-lam2)*right-(Gy-mu2)*up;
P30 = G0-thickness/2*front-(Gx-lam3)*right-(Gy-mu3)*up;
B0  = G0-thickness/2*front-(Gx-beta1)*right-(Gy-alpha1)*up;
R = [1,0,0;0,cos(psi),-sin(psi);0,sin(psi),cos(psi)]*[cos(phi),0,sin(phi);0,1,0;-sin(phi),0,cos(phi)]*[cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
P1 = G + R*(P10-G0);
P2 = G + R*(P20-G0);
P3 = G + R*(P30-G0);
B  = G + R*(B0-G0);

%% Plotting
figure(figure_number);hold on;
plot3(P1(1),P1(2),P1(3),'b.','MarkerSize',20);
plot3(P2(1),P2(2),P2(3),'b.','MarkerSize',20);
plot3(P3(1),P3(2),P3(3),'b.','MarkerSize',20);
plot3(B(1),B(2),B(3),'g.','MarkerSize',20);


end