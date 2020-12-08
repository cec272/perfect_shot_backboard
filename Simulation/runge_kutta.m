% Harrison Hidalgo
% ECE 5725 - Final Project
% Runge-Kutta 4th order Integrator
%

function [t_next,x_next] = runge_kutta(del_t,t,x,ffun,p)
    k1 = feval(ffun,t,x,p);
    k2 = feval(ffun,t+del_t/2,x+del_t*k1/2,p);
    k3 = feval(ffun,t+del_t/2,x+del_t*k2/2,p);
    k4 = feval(ffun,t+del_t,x+del_t*k3,p);
    x_next = x + del_t/6*(k1+2*k2+2*k3+k4);
    t_next = t + del_t;
end