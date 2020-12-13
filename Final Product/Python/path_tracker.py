# Harrison Hidalgo
# ECE 5725 - Final Project
# This function tells if the ball goes through the hoop from its initial 
# states and the backboard's current orientation.

def does_it_go_through = path_tracker(h,x_ball_init,ffun,p,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop,backboard):
    from runge_kutta import *
    from collision import *
    does_it_go_through = 0;
    t_new=0
    x_new=x_ball_init
    collision_tracker=0
    while (t_new < 2)
        t_new,x_new = runge_kutta(h,t_new,x_new,ffun,p);
        if (collision_tracker < 1) && did_it_collide(backboard,x_new[0:3],front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball)
            x_new(3:6) = collision(up,front,x_new[3:6],p.e);
            collision_tracker=collision_tracker+1;
        elif (collision_tracker < 5)&&(collision_tracker>0)
            collision_tracker=collision_tracker+1;
        elif (collision_tracker > 5)
            collision_tracker = 0;
        if ball_in_hoop(x_new[0:3],r_of_ball,hoop_position,center_hoop)
            does_it_go_through = 1;
            return
 
def is_it = ball_in_hoop(ball_location,ball_radius,hoop_position)
# Distance between points
r_ball_hoop = hoop_position - ball_location;
# Compare to radius
if abs(norm(r_ball_hoop)-ball_radius) < 1.25*ball_radius
    is_it = 1;
else
    is_it = 0;
 
