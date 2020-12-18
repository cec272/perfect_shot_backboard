# Harrison Hidalgo
# ECE 5725 - Final Project 
# Square-Root Sigma Point Kalman Filter Ball

### Done before: initialize

import numpy as np
from ball_calc import *
from runge_kutta import *
import weights
import math

def SR_SPF_Ball(x_0,S_x0,S_v0,S_n0,n_sig,measurement,dt):
  # S_v0 is process noise
  # S_n0 is sensor noise
  H = np.identity(6)
  ## Create sigma points
  nx = len(x_0)
  ensp = np.ones((1,nx*2+1));
  sigma_points=np.matmul(x_0,ensp)+np.concatenate((np.zeros((nx,1)),n_sig*S_x0,-n_sig*S_x0),axis=1);
  ## Predict
  X_p1 = np.zeros((nx,nx*2+1))
  time,X_p1[:,None,0]=runge_kutta(dt,0,sigma_points[:,None,0])
  x_p1=X_p1[:,None,0]*weights.wm0;
  for i in range(0,nx):
    time,X_p1[:,None,i+1] = runge_kutta(dt,0,sigma_points[:,None,1+i]);
    time,X_p1[:,None,i+1+nx] = runge_kutta(dt,0,sigma_points[:,None,1+i+nx]);
    x_p1 = weights.wm*(X_p1[:,None,i+1]+X_p1[:,None,i+1+nx])+x_p1;
  B_k=np.zeros((2*nx,nx));
  for i in range(0,2*nx):
    B_k[i,None,:] = np.sqrt(weights.wc)*np.transpose(X_p1[:,None,i+1]-x_p1)
  B_k = np.concatenate((B_k,np.transpose(S_v0)),axis=0)
  S_x1,r = np.linalg.qr(np.transpose(B_k))
  #S_x1 = np.linalg.cholesky(S_x1+np.sign(weights.wc0)*np.matmul((X_p1[:,None,0]-x_p1),np.transpose(X_p1[:,None,0]-x_p1)))
  S_x1 = cholupdate(S_x1,X_p1[:,None,0]-x_p1,np.sign(weights.wc0))
  print('chol 1')
  print(S_x1)
  X_p2 = np.zeros((nx,nx*2+1))
  X_p2[:,None,0]=x_p1;
  ## Re create sigma points
  X_p2=np.matmul(x_p1,ensp)+np.concatenate((np.zeros((nx,1)),n_sig*S_x1,-n_sig*S_x1),axis=1);
  #### Update
  Y_k=np.zeros((nx,2*nx+1));
  Y_k[:,None,0] = np.matmul(H,X_p2[:,None,0])
  y_k = Y_k[:,None,0]*weights.wm0
  for i in range(1,2*nx+1):
    Y_k[:,None,i]= np.matmul(H,X_p2[:,None,i])
    y_k=y_k+Y_k[:,None,i]*weights.wm;
  C_k=np.zeros((2*nx,nx))
  for i in range(0,2*nx):
    C_k[i,None,:]=weights.wc*np.transpose((Y_k[:,None,i+1]-y_k));
  C_k=np.concatenate((C_k,np.transpose(S_n0)),axis=0);
  #print('C_k')
  #print(C_k)
  S_yk,r=np.linalg.qr(np.transpose(C_k));
  #S_yk=np.linalg.cholesky(S_yk+np.matmul((Y_k[:,None,0]-y_k),np.transpose((Y_k[:,None,0]-y_k)))*np.sign(weights.wc0))
  S_yk=cholupdate(S_yk,Y_k[:,None,0]-y_k,np.sign(weights.wc0));
  P_xy = weights.wc0*np.matmul((X_p2[:,None,0]-x_p1),np.transpose(Y_k[:,None,0]-y_k))
  for i in range(0,2*nx+1):
    P_xy=weights.wc* np.matmul((X_p2[:,None,i]-x_p1),np.transpose(Y_k[:,None,i]-y_k))+P_xy
  Kalman_Gain=np.matmul(np.matmul(P_xy,np.transpose(S_yk)),S_yk);
  innovation=measurement-np.matmul(H,x_p1)
  x_hat=x_p1+np.matmul(Kalman_Gain,innovation)
  KF_gain_cov = np.matmul(Kalman_Gain,P_xy)
  #S_xk = np.linalg.cholesky(S_x1+np.matmul(KF_gain_cov[:,None,0],np.transpose(KF_gain_cov[:,None,0]))*-1)
  S_xk = cholupdate(S_x1,KF_gain_cov[:,None,0],-1)
  for i in range(1,nx):
    S_xk = cholupdate(S_xk,KF_gain_cov[:,None,i],-1);
  return x_hat,S_xk

def cholupdate(R,x,sign):
  #R = validateCovMatrix(R)
  p = len(x)
  for k in range(0,p):
      #print('first')
      #print(R[k,None,k]**2)
      #print('sec')
      #print(sign*x[k]**2)
    r = np.sqrt(R[k,None,k]**2 + sign*x[k]**2)
    c = r/R[k,None,k]
    s = x[k]/R[k,None,k]
    R[k,None,k] = r
    if k<p:
      R[(k+1):p,None,k] = (R[k+1:p,None,k] + sign*s*x[(k+1):p])/c
      x[(k+1):p]= c*x[(k+1):p] - s*R[(k+1):p,None,k]
  R = validateCovMatrix(R)
  return R

def validateCovMatrix(sig):
  EPS = 10**-6
  ZERO= 10**-10
  sigma = sig
  if (is_pos_def(sigma) == False):
    w,v = np.linalg.eig(sigma)
    for n in range(0,len(w)):
      if (v[n,n] <= ZERO):
        v[n,n] = EPS
      sigma = w*v*np.transpose(w)
  return sigma
    
def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) > 0)
