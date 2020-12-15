# Harrison Hidalgo
# ECE 5725 - Final Project
# This function tells if the ball goes through the hoop from its initial 
# states and the backboard's current orientation.

def path_tracker(h,x_ball_init,front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball,center_hoop,backboard):
    import runge_kutta
    import Physical_Variables as p
    does_it_go_through = False
    t_new=0
    x_new=x_ball_init
    collision_tracker=0
    while (t_new < 0.75):
        t_new,x_new = runge_kutta.runge_kutta(h,t_new,x_new);
        location, does_it_collide = did_it_collide(backboard,x_new[0:3],front,up,W_of_backboard,H_of_backboard,T_of_backboard,r_of_ball)
        if (collision_tracker < 1) and does_it_collide:
            location,x_new[3:6] = collision(up,front,x_new[3:6],p.e);
            collision_tracker=collision_tracker+1;
            does_it_collide = True
            break
        elif (collision_tracker < 5)and(collision_tracker>0):
            collision_tracker=collision_tracker+1;
        elif (collision_tracker > 5):
            collision_tracker = 0;
        if ball_in_hoop(x_new[0:3],r_of_ball,center_hoop):
            does_it_go_through = True
            return does_it_go_through
    return does_it_collide,location
 
def ball_in_hoop(ball_location,ball_radius,hoop_position):
    import numpy as np
    # Distance between points
    r_ball_hoop = hoop_position - ball_location;
    # Compare to radius
    if abs(np.linalg.norm(r_ball_hoop)-ball_radius) < 1.25*ball_radius:
        is_it = 1
    else:
        is_it = 0
    return is_it

def collision(up,front,velocity,restitution):
    import numpy as np
    # Calculate the left component of the backboard.
    left = np.cross(front,up);
    # Decompose velocity into backboard coordinates.
    up_comp = np.dot(velocity,up)/np.dot(up,up)*up;
    front_comp = np.dot(velocity,front)/np.dot(front,front)*front;
    left_comp = np.dot(velocity,left)/np.dot(left,left)*left;
    # Put velocity back together.
    velocity_new = up_comp-front_comp*restitution+left_comp;
    return velocity_new

def did_it_collide(G,B,eF,eU,W,H,T,R):
    # This figures out if the ball collided with the backboard
    # Parameters:
    # G  = Backboard CoM
    # B  = Ball position
    # eF = unit vector front backboard (i)
    # eU = unit vector up backboard (j)
    # W = width of backboard
    # H = height of backboard
    # T = thickness of backboard
    # R = radius of ball
    import numpy as np
    P = G + T/2*eF
    N = 101
    eL = np.transpose(np.cross(np.transpose(eF),np.transpose(eU)));
    LR=float('inf')*np.ones((N,1))
    DU=float('inf')*np.ones((N,1))
    width = np.linspace(-W/2,W/2,N)
    height = np.linspace(-H/2,H/2,N)
    Dir1 = np.ones((3,N))
    Dir2 = np.ones((3,N))
    for i in range(0,N):
        Dir1[:,i] = P + width[i]*eL;
        Dir2[:,i] = P + height[i]*eU;
        LR[i] = np.linalg.norm(Dir1[:,i]-B)
        DU[i] = np.linalg.norm(Dir1[:,i]-B)
    LRmin = np.where(LR==min(LR))
    DUmin = np.where(DU==min(DU))
    min_distance = np.linalg.norm(np.squeeze(Dir1[:,LRmin[0]])+np.squeeze(Dir2[:,DUmin[0]])-P-B);
    if min_distance < R:
        IsIt = True
    else:
        IsIt = False
    Location = np.squeeze(Dir1[:,LRmin[0]])+np.squeeze(Dir2[:,DUmin[0]])-P-B
    return Location,IsIt
