function M = A_backboard(Gx,Gy,I1,I2,I3,alpha1,beta1,front1,front2,front3,m,phi,psi,theta,thickness,up1,up2,up3)
%A_BACKBOARD
%    M = A_BACKBOARD(GX,GY,I1,I2,I3,ALPHA1,BETA1,FRONT1,FRONT2,FRONT3,M,PHI,PSI,THETA,THICKNESS,UP1,UP2,UP3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    26-Nov-2020 21:26:06

t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = front1.*up2;
t9 = front2.*up1;
t10 = front1.*up3;
t11 = front3.*up1;
t12 = front2.*up3;
t13 = front3.*up2;
t15 = -alpha1;
t16 = -beta1;
t24 = (front1.*thickness)./2.0;
t25 = (front2.*thickness)./2.0;
t26 = (front3.*thickness)./2.0;
t14 = t2.^2;
t17 = t3.*t4;
t18 = t3.*t7;
t19 = t4.*t6;
t20 = t6.*t7;
t21 = -t9;
t22 = -t11;
t23 = -t13;
t27 = Gy+t15;
t28 = Gx+t16;
t33 = I1.*t2.*t4.*t5;
t37 = I1.*t2.*t5.*t7;
t29 = t5.*t20;
t30 = t5.*t17;
t31 = t5.*t18;
t32 = t5.*t19;
t34 = t27.*up1;
t35 = t27.*up2;
t36 = t27.*up3;
t38 = t8+t21;
t39 = t10+t22;
t40 = t12+t23;
t43 = -t33;
t44 = -t37;
t45 = I1.*t4.*t7.*t14;
t41 = -t31;
t42 = -t32;
t46 = t17+t29;
t47 = t20+t30;
t50 = t28.*t38;
t51 = t28.*t39;
t52 = t28.*t40;
t48 = t18+t42;
t49 = t19+t41;
t53 = -t51;
t54 = I3.*t2.*t3.*t47;
t55 = I2.*t2.*t6.*t46;
t62 = t24+t34+t52;
t63 = t26+t36+t50;
t56 = I3.*t2.*t3.*t49;
t57 = I2.*t2.*t6.*t48;
t60 = I2.*t46.*t48;
t61 = I3.*t47.*t49;
t66 = t25+t35+t53;
t67 = t2.*t4.*t62;
t68 = t2.*t7.*t62;
t70 = t47.*t63;
t72 = t49.*t63;
t58 = -t56;
t59 = -t57;
t64 = -t60;
t65 = -t61;
t69 = -t67;
t71 = t46.*t66;
t73 = -t70;
t74 = t48.*t66;
t75 = -t72;
t76 = t43+t54+t59;
t77 = t44+t55+t58;
t78 = t45+t64+t65;
t79 = t68+t71+t75;
t80 = t69+t73+t74;
M = reshape([m,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,m,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,m,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I2.*t48.^2+I3.*t47.^2+I1.*t4.^2.*t14,t78,t76,t79,t80,0.0,0.0,0.0,t78,I2.*t46.^2+I3.*t49.^2+I1.*t7.^2.*t14,t77,t4.*t5.*t62-t2.*t17.*t63-t2.*t19.*t66,t5.*t7.*t62-t2.*t18.*t63-t2.*t20.*t66,0.0,0.0,0.0,t76,t77,I1.*t5.^2+I3.*t3.^2.*t14+I2.*t6.^2.*t14,-t48.*t63-t47.*t66,t46.*t63+t49.*t66,0.0,-1.0,0.0,t5.*t62-t2.*t3.*t63-t2.*t6.*t66,0.0,t67+t70-t74,0.0,0.0,0.0,0.0,-1.0,t79,t80,0.0,0.0,0.0],[8,8]);