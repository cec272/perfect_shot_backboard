% Harrison Hidalgo
% ECE 5725 - Final Project 
% Square-Root Sigma Point Kalman Filter Backboard
%

%%% Done before: initialize

function [x_hat,S_xk] = SR_SPF_Backboard(x_0,S_x0,S_v0,S_n0,n_sig,weights,measurement)
%% Create sigma points
sigma_points = zeros(length(x_0),n_sig*2+1);
sigma_points(:,1)=x_0;
for i=1:n_sig
sigma_points(:,1+i)=x_0+n_sig*S_x0;
sigma_points(:,1+i+sig_n)=x_0-n_sig*S_x0;
end
%% Predict
X_p1(:,1)=backboard_calc(0,sigma_point(1),p);x_p1=X_p1(:,1)*weights.wm0;
for i=1:n_sig
X_p1(:,i+1) = backboard_calc(0,sigma_point(1+i),p);
X_p1(:,i+1+n_sig) = backboard_calc(0,sigma_point(1+i+n_sig),p);
x_p1 = weights.wm*(X_p1(:,i+1)+X_p1(:,i+1+n_sig))+x_p1;
end
B_k=zeros(2*n_sig+1,length(x_0));
for i = 1:2*nsig
B_k(i,:) = sqrt(weights.wc)*(X_p1(:,i+1)-x_p1)';
end
B_k(2*n_sig+1,:) = S_v0';
S_x1 = qr(B_k');
S_x1 = cholupdate(S_x1,X_p1(1)-x_p1,sgn(weights.wc0));
X_p2=zeros(length(x_0),n_sig*2+1);X_p2(:,1)=x_p1;
for i=1:n_sig
X_p2(:,i+1)=x_p1+n_sig*S_x1;
X_p2(:,i+1+n_sig)=x_p1-n_sig*S_x1;
end
%% Update
Y_k=zeros(length(x_0),2*n_sig+1);
for i=1:2*n_sig
Y_k(:,i)= ;% ***************
y_k=y_k+Y_k(:,i)*weights.wm;
end
C_k=zeros(2*n_sig+1,length(x_0));%%%%%%%%%%
for i=1:2*n_sig
C_k(i,:)=weights.wc*(Y_k(:,i+1)-y_k)');
end
C_k(2*n_sig+1,:)=S_n0';
S_yk=qr(C_k);
S_yk=cholupdate(S_yk,Y_k(:,1)-y_k,sgn(weights.wc));
P_xy = zeros(length(x_0),length(x_0),2*n_sig+1);
for i=1:2*n_sig+1
P_xy(:,:,i)=weights.wc*(X_p2(:,i)-x_p1)*(Y_k(:,i)-y_k)';
end
Kalman_Gain=(P_xy/S_yk')/S_yk;
innovation=measurement-;%%%%%%%%%%%%%%%%%%%
x_hat=x_p1+Kalman_Gain*innovation;
S_xk = cholupdate(S_x1,Kalman_Gain*S_yk,-1);
end