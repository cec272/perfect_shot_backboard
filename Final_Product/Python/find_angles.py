# Harrison Hidalgo
# ECE 5725 - Final Project
# This program parses through possible combos of theta1 theta2 and theta3
# to find necessary combo.

def find_angles(theta,phi,psi,r_B):
    import numpy as np
    import transformations
    import math
    import iteration_variables as IV
    import Geometric_Variables as GV
    
    theta_1 = np.array([])
    theta_2 = np.array([])
    theta_3 = np.array([])
    
    N=5;
    th1=np.linspace(IV.th1_start,IV.th1_end,N);
    th2=np.linspace(IV.th2_start,IV.th2_end,N);
    th3=np.linspace(IV.th3_start,IV.th3_end,N);
    rP1= r_B-transformations.transform_baseboard_to_backboard(theta,phi,psi,(GV.rGB0+GV.rGP10))
    rP2= r_B-transformations.transform_baseboard_to_backboard(theta,phi,psi,(GV.rGB0+GV.rGP20))
    rP3= r_B-transformations.transform_baseboard_to_backboard(theta,phi,psi,(GV.rGB0+GV.rGP30))
    does_it_work=False
    for i in range(0,N):
        for j in range(0,N):
            for k in range(0,N):
                th2_1=th1[i]
                th2_2=th2[j]
                th2_3=th3[k]
                # Rotation about an arbitrary axis
                motor_rod1 = np.dot(GV.motor_rod10,GV.motor_up1)*GV.motor_up1+math.cos(th2_1)*GV.motor_rod10+math.sin(th2_1)*np.cross(GV.motor_up1,GV.motor_rod10);
                motor_rod2 = np.dot(GV.motor_rod20,GV.motor_up2)*GV.motor_up2+math.cos(th2_2)*GV.motor_rod20+math.sin(th2_2)*np.cross(GV.motor_up2,GV.motor_rod20);
                motor_rod3 = np.dot(GV.motor_rod30,GV.motor_up3)*GV.motor_up3+math.cos(th2_3)*GV.motor_rod30+math.sin(th2_3)*np.cross(GV.motor_up3,GV.motor_rod30);

                # Motor points
                motor_point_1 = GV.gear2_1+motor_rod1
                motor_point_2 = GV.gear2_2+motor_rod2
                motor_point_3 = GV.gear2_3+motor_rod3
                # Rods
                r_P1_MP1 = motor_point_1 - rP1;
                r_P2_MP2 = motor_point_2 - rP2;
                r_P3_MP3 = motor_point_3 - rP3;

                if (np.linalg.norm(r_P1_MP1-rP1)-GV.L1 < 0.1)and(np.linalg.norm(r_P2_MP2-rP2)-GV.L2 < 0.1)and(np.linalg.norm(r_P3_MP3-rP3)-GV.L3 < 0.1):
                    does_it_work=True
                    theta_1 = th2_1
                    theta_2 = th2_2
                    theta_3 = th2_3
                    break
    return does_it_work,theta_1,theta_2,theta_3
