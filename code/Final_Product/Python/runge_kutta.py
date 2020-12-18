# Harrison Hidalgo
# ECE 5725 - Final Project
# Runge-Kutta 4th order Integrator
#

def runge_kutta(del_t,t0,x):
    import ball_calc
    k1 = ball_calc.ball_calc(x)
    k2 = ball_calc.ball_calc(x+del_t*k1/2)
    k3 = ball_calc.ball_calc(x+del_t*k2/2)
    k4 = ball_calc.ball_calc(x+del_t*k3)
    x_next = x + del_t/6*(k1+2*k2+2*k3+k4)
    t_next = t0 + del_t
    return t_next,x_next
