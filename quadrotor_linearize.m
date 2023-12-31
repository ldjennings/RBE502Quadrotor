function [A,B] = quadrotor_linearize(in1,in2,in3)
%Linearize
%    [A,B] = Linearize(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    28-Oct-2023 18:10:20

Ixx = in3(:,4);
Iyy = in3(:,5);
Izz = in3(:,6);
W1 = in1(10,:);
W2 = in1(11,:);
W3 = in1(12,:);
l = in3(:,2);
m = in3(:,3);
phi = in1(4,:);
psi = in1(6,:);
sigma = in3(:,8);
theta = in1(5,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
t9 = Ixx.*W1;
t10 = Iyy.*W2;
t11 = Izz.*W3;
t13 = 1.0./Ixx;
t14 = 1.0./Iyy;
t15 = 1.0./Izz;
t16 = 1.0./m;
t19 = u1+u2+u3+u4;
t12 = t8.^2;
t17 = t3.*t5;
t18 = t5.*t6;
t20 = 1.0./t4;
t23 = l.*t13;
t24 = l.*t14;
t25 = sigma.*t15;
t27 = t2.*t3.*t7;
t28 = t2.*t6.*t7;
t30 = t2.*t4.*t16;
t21 = t20.^2;
t26 = t12+1.0;
t29 = -t25;
t31 = -t28;
t32 = t18+t27;
t33 = t17+t31;
t34 = t16.*t32;
t35 = t16.*t33;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,W2.*t2.*t8-W3.*t5.*t8,-W3.*t2-W2.*t5,W2.*t2.*t20-W3.*t5.*t20,t16.*t19.*(t2.*t6-t7.*t17),-t16.*t19.*(t2.*t3+t7.*t18),-t4.*t5.*t16.*t19,0.0,0.0,0.0,0.0,0.0,0.0,W3.*t2.*t26+W2.*t5.*t26,0.0,W3.*t2.*t7.*t21+W2.*t5.*t7.*t21,t3.*t19.*t30,t6.*t19.*t30,-t2.*t7.*t16.*t19,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t19.*t35,t19.*t34,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t14.*(t11-Ixx.*W3),-t15.*(t10-Ixx.*W2),0.0,0.0,0.0,t5.*t8,t2,t5.*t20,0.0,0.0,0.0,-t13.*(t11-Iyy.*W3),0.0,t15.*(t9-Iyy.*W1),0.0,0.0,0.0,t2.*t8,-t5,t2.*t20,0.0,0.0,0.0,t13.*(t10-Izz.*W2),-t14.*(t9-Izz.*W1),0.0];
A = reshape([mt1,mt2],12,12);
if nargout > 1
    t36 = -t35;
    B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t34,t36,t30,0.0,-t24,t25,0.0,0.0,0.0,0.0,0.0,0.0,t34,t36,t30,t23,0.0,t29,0.0,0.0,0.0,0.0,0.0,0.0,t34,t36,t30,0.0,t24,t25,0.0,0.0,0.0,0.0,0.0,0.0,t34,t36,t30,-t23,0.0,t29],[12,4]);
end
end
