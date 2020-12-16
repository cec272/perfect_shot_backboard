% Harrison Hidalgo 
% ECE 5725 - Final Project
% This function determines if a collision has occured. 
%

function IsIt = did_it_collide(G,B,eF,eU,W,H,T,R)
P = G + T/2*eF;
N = 100; eL = cross(eF,eU);
Dir1 = P + linspace(-W/2,W/2,N) .* eL;
Dir2 = P + linspace(-H/2,H/2,N) .* eU;
LR=inf*ones(N,1);DU=inf*ones(N,1);
for i = 1:N
    LR(i) = norm(Dir1(:,i)-B);
    DU(i) = norm(Dir1(:,i)-B);
end
LRmin = find(LR==min(LR));DUmin = find(DU==min(DU));
min_distance = norm(Dir1(:,LRmin)+Dir2(:,DUmin)-P-B);
if min_distance < R
    IsIt = true;
else
    IsIt = false;
end
end