# Harrison Hidalgo
# ECE 5725 - Final Project 
# Square-Root Sigma Point Kalman Filter Ball

### Done before: initialize

import numpy as np
from ball_calc import * 
import weights

def SR_SPF_Ball(x_0,S_x0,S_v0,S_n0,n_sig,measurement,dt):
  # S_v0 is process noise
  # S_n0 is sensor noise
  H = np.identity(6)
  ## Create sigma points
  nx = len(x_0)
  ensp = np.ones((1,nx*2+1));
  sigma_points=np.matmul(x_0,ensp)+np.concatenate((np.zeros((nx,1)),-n_sig*S_x0,n_sig*S_x0),axis=1);

  ## Predict
  X_p1 = np.zeros((nx,nx*2+1))
  X_p1[:,0]=ball_calc(sigma_points[:,0])
  x_p1=X_p1[:,0]*weights.wm0;
  for i in range(0,nx):
    X_p1[:,i+1] = dt*ball_calc(sigma_points[:,1+i]);
    X_p1[:,i+1+nx] = dt*ball_calc(sigma_points[:,1+i+nx]);
    x_p1 = weights.wm*(X_p1[:,i+1]+X_p1[:,i+1+n_sig])+x_p1;
  B_k=np.zeros((2*nx,nx));
  for i in range(0,2*nx):
    B_k[i,:] = np.sqrt(weights.wc)*np.transpose((X_p1[:,i+1]-x_p1))
  B_k = np.concatenate((B_k,np.transpose(S_v0)),axis=0)
  S_x1,r = np.linalg.qr(np.transpose(B_k))
  S_x1 = cholupdate(S_x1,X_p1[:,0]-x_p1,np.sign(weights.wc0))
  X_p2 = np.zeros((nx,nx*2+1))
  X_p2[:,0]=x_p1;
  ## Re create sigma points
  X_p2=np.matmul(np.transpose(np.array([x_p1])),ensp)+np.concatenate((np.zeros((nx,1)),-n_sig*S_x1,n_sig*S_x1),axis=1);

  #### Update
  Y_k=np.zeros((nx,2*nx+1));
  y_k = 0
  for i in range(0,2*nx):
    Y_k[:,i]= np.matmul(H,X_p2[:,i])
    y_k=y_k+Y_k[:,i]*weights.wm;
  C_k=np.zeros((2*nx,nx))
  for i in range(0,2*nx):
    C_k[i,:]=weights.wc*np.transpose((Y_k[:,i+1]-y_k));
  C_k=np.concatenate((C_k,np.transpose(S_n0)),axis=0);
  S_yk,r=np.linalg.qr(np.transpose(C_k));
  S_yk=cholupdate(S_yk,Y_k[:,0]-y_k,np.sign(weights.wc));
  P_xy = np.zeros((nx,nx));
  for i in range(0,2*nx+1):
    P_xy=weights.wc*(X_p2[:,i]-x_p1)*np.transpose(Y_k[:,i]-y_k)+P_xy
  Kalman_Gain=P_xy/np.transpose(S_yk)/S_yk;
  innovation=measurement-np.array([x_p1])
  x_hat=np.transpose(np.array([x_p1]))+np.matmul(Kalman_Gain,np.transpose(innovation))
  S_xk = cholupdate(S_x1,x_p1-np.matmul(Kalman_Gain,y_k),-1);
  return x_hat,S_xk

def cholupdate(R,x,sign):
    import numpy as np
    p = len(x)
    x = x.T
    for k in range(0,p):
      r = np.sqrt(R[k,k]**2 + sign*x[k]**2)
      c = r/R[k,k]
      s = x[k]/R[k,k]
      R[k,k] = r
      if k<p:
        R[k,k+1:p] = (R[k,k+1:p] + sign*s*x[k+1:p])/c
        x[k+1:p]= c*x[k+1:p] - s*R[k,k+1:p]
    return R

#def cholupdate2(A,B,sign):
#  for i in range(0,6):
#    alpha = 1
#    for j = 
  
