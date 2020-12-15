# Harrison Hidalgo
# ECE 5725 - Final Project 
# Square-Root Sigma Point Kalman Filter Ball

### Done before: initialize

def SR_SPF_Ball(x_0,S_x0,S_v0,S_n0,n_sig,p,measurement,dt):
  # S_v0 is process noise
  # S_n0 is sensor noise
  import numpy as np
  from ball_calc import * 
  import weights
  H = np.identity(6)
  ## Create sigma points
  nx = len(x_0)
  ensp = np.ones(1,n_sig*2+1);
  sigma_points=numpy.matmul(x_0,ensp)+np.concatenate(np.zeros(nx),-S_x0,S_x0);

  ## Predict
  X_p1[:,0]=ball_calc(0,sigma_points[:,0],p)
  x_p1=X_p1[:,0]*weights.wm0;
  for i in range(0,n_sig):
    X_p1[:,i+1] = dt*ball_calc(0,sigma_points[:,1+i],p);
    X_p1[:,i+1+n_sig] = dt*ball_calc(0,sigma_point[:,1+i+n_sig],p);
    x_p1 = weights.wm*(X_p1[:,i+1]+X_p1[:,i+1+n_sig])+x_p1;

  B_k=np.zeros(2*n_sig+1,nx);
  for i in range(0,2*n_sig):
    B_k[i,:] = sqrt(weights.wc)*np.transpose((X_p1[:,i+1]-x_p1))
  B_k[2*n_sig,:] = np.transpose(S_v0)
  S_x1 = np.linalg.qr(np.transpose(B_k))
  S_x1 = cholupdate(S_x1,X_p1[:,0]-x_p1,np.sign(weights.wc0))
  X_p2 = np.zeros(nx,n_sig*2+1)
  X_p2[:,0]=x_p1;
  ## Re create sigma points
  X_p2=numpy.matmul(x_p1,ensp)+np.concatenate(np.zeros(nx),-S_x1,S_x1);

  #### Update
  Y_k=np.zeros(nx,2*n_sig+1);
  for i in range(0,2*n_sig):
    Y_k[:,i]= np.matmul(H,X_p2[:,i])
    y_k=y_k+Y_k[:,i]*weights.wm;

  C_k=np.zeros(2*n_sig+1,nx)
  for i in range(0,2*n_sig):
    C_k[i,:]=weights.wc*np.transpose((Y_k[:,i+1]-y_k));
  C_k[2*n_sig+1,:]=np.transpose(S_n0);
  S_yk=np.linalg.qr(C_k);
  S_yk=cholupdate(S_yk,Y_k[:,0]-y_k,np.sign(weights.wc));
  P_xy = np.zeros(nx,nx,2*n_sig+1);
  for i in range(0,2*n_sig+1):
    P_xy[:,:,i]=weights.wc*(X_p2[:,i]-x_p1)*np.transpose(Y_k[:,i]-y_k)

  Kalman_Gain=(np.squeeze(P_xy)/np.transpose(S_yk))/S_yk;
  innovation=measurement-np.matmul(H,x_p1)
  x_hat=x_p1+np.matmul(Kalman_Gain,innovation)
  S_xk = cholupdate(S_x1,np.matmul(Kalman_Gain,S_yk),-1);
  return x_hat,S_xk

def cholupdate(R,x,sign):
    import numpy as np
    p = np.size(x)
    x = x.T
    for k in range(p):
      if sign == '+':
        r = np.sqrt(R[k,k]**2 + x[k]**2)
      elif sign == '-':
        r = np.sqrt(R[k,k]**2 - x[k]**2)
      c = r/R[k,k]
      s = x[k]/R[k,k]
      R[k,k] = r
      if sign == '+':
        R[k,k+1:p] = (R[k,k+1:p] + s*x[k+1:p])/c
      elif sign == '-':
        R[k,k+1:p] = (R[k,k+1:p] - s*x[k+1:p])/c
      x[k+1:p]= c*x[k+1:p] - s*R[k, k+1:p]
    return R
