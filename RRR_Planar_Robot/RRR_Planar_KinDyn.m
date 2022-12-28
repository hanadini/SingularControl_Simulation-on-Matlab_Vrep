% function [Mq,hq,J,dJ,P,C,x,Js,CN] = RRR_Planar_KinDyn(q)%Omni_KinDyn(q)  %function [Mq,Cq,Gq,J,dJ,x,Js] = RRR_Planar_KinDyn(q,MProp,LProp)
function [Mq,hq,J,dJ,x,Js,CN] = RRR_Planar_KinDyn(q)
%==========================================================================
%Compute kinematic & dynamics parameters of an omni manipulator

%==========================================================================

%Set gravitional acceleration
g = 9.81;
%g = 0;
%Read physical properties of manipulator

%for the first link:l1&m1  2nd link:l2&m3  3rd link:l3&m1
%m1 = MProp(1);m2 = MProp(2); m3 = MProp(3); %get link masses
 m2 = 0.035; m3 = 0.1;m1 = 0.35;

%l1 = LProp(1);l2 = LProp(2); l3 = LProp(3); %get link lengths
 l1 = 0.135; l2 = 0.135; l3 = 0;


 % joint angles correction
  %q(3) = q(3)-(pi/2);
%==========================================================================
%Define some trigonometric functions

c1 = cos(q(1));         c2 = cos(q(2));         c3 = cos(q(3));
s1 = sin(q(1));         s2 = sin(q(2));         s3 = sin(q(3));
c12 = cos(q(1)+q(2));   s12 = sin(q(1)+q(2));
c23 = cos(q(2)+q(3));   s23 = sin(q(2)+q(3));
c123 = cos(q(1)+q(2)+q(3));
s123 = sin(q(1)+q(2)+q(3));

c2_2 = cos(2*q(2));     c2_23 = cos(2*(q(2)+q(3))); c22_3 = cos(2*q(2)+q(3));
s2_2 = sin(2*q(2));     s2_23 = sin(2*(q(2)+q(3))); s22_3 = sin(2*q(2)+q(3)); 

%Compute equation of motion matrices in joint space
Ixx1 = 5.040e-04;   Iyy1 = 5.040e-04;        Izz1 = 5.040e-04;
Ixx2 = 0;           Iyy2 = 2.126e-4;         Izz2 = 2.126e-4;
Ixx3 = 0;           Iyy3 = 6.075e-04;        Izz3 = 6.075e-04;

m11 = (1/8)*(8*(s2^2)*Ixx2 + 8*(s23^2)*Ixx3 + 4*Iyy2*(1+c2_2) + ...
    4*Iyy3*(1+c2_23) + 8*Izz1 +(l1^2)*m2*(1 + c2_2) + ...
    4*(l1^2)*m3*(1+c2_2) + 4*l1*l2*m3*(c3+c22_3) + (l2^2)*m3*(1+c2_23));
m12 = 0;
m13 = 0;
m21 = 0;
m22 = (1/8)*(2*(4*Izz2+4*Izz3+l1^2*m2+4*m3*l1^2+4*c3*l1*l2*m3 + (l2^2)*m3));
m23 = (1/8)*2*(4*Izz3 + l2*m3*(2*c3*l1 + l2));
m31 = 0;
m32 = (1/4)*(4*Izz3 + l2*m3*(2*c3*l1+l2));
m33 = (1/4)*(4*Izz3+m3*(l2^2));

Mq = [m11 m12 m13;m21 m22 m23;m31 m32 m33];

%h=b+g

h1 = (1/8)*(2*q(4)*(q(5)*(4*s2_2*(Ixx2 - Iyy2) + 4*s2_23*(Ixx3 - ...
    Iyy3) - s2_2*(l1^2)*(m2 +4*m3) - 4*s22_3*l1*l2*m3 - s2_23*(l2^2)*m3))+...
    4*s23*q(4)*q(6)*(c23*(4*Ixx3 - 4*Iyy3 +l2^2*m3)-2*l1*l2*m3*c2));


% h2 = (1/8)*((q(4)^2)*(4*s2_2*(Iyy2 - Ixx2) + 4*s2_23*(Iyy3 - Ixx3) + ...
%     s2_2*(l1^2)*(m2 + 4*m3) + 4*s22_3*l1*l2*m3 + s2_23*(l2^2)*m3) - ...
%     4*(g*c23*l2*m3 +l1*g*c2*m2 + 2*l1*g*m3^2*c2) + 8*q(5)*q(6)*s3*l1*l2*m3 + 4*l1*l2*m3*s3*q(6)^2);

h2 = (1/8)*((q(4)^2)*(4*s2_2*(Iyy2 - Ixx2) + 4*s2_23*(Iyy3 - Ixx3) + ...
    s2_2*(l1^2)*(m2 + 4*m3) + 4*s22_3*l1*l2*m3 + s2_23*(l2^2)*m3) - ...
    4*(g*c23*l2*m3 +l1*g*c2*m2 + 2*l1*g*m3*c2) + 8*q(5)*q(6)*s3*l1*l2*m3 + 4*l1*l2*m3*s3*q(6)^2);


h3 = (1/4)*(-2*l2*m3*g*c23 + 2*s3*(q(5)^2)*l1*l2*m3 - s23*(q(4)^2)*(4*c23*(Ixx3-Iyy3) - ...
    -l2*m3*(2*c2*l1+c23*l2)));


hq= [h1;h2;h3];

%==========================================================================
%Perform Jacobian computations

R01=[c1 -s1 0;s1 c1 0;0 0 1];
R12=[c2 -s2 0;0 0 1;-s2 -c2 0];
R23=[c3 -s3 0;s3 c3 0;0 0 1];
R3s=[1 0 0;0 1 0;0 0 1];
R0s=R01*R12*R23*R3s;

%Here ,J & dJ,denoted Linear Jacobian and its derivative:

J = [-s1*(l1*c2+l2*c23),  -c1*(l1*s2+l2*s23),   -l2*c1*s23;
         c1*(l1*c2+l2*c23),-s1*(l1*s2+l2*s23),   -l2*s1*s23;
        %1,2,   1;
        0,                -(l1*c2+l2*c23),      -l2*c23];

[U,S,V] = svd(J);
[det(J),diag(S).'];
CN = cond(J);

Js = R0s*J;


dJ = [-c1*q(4)*(l1*c2+l2*c23)-s1*(-l1*s2*q(5)-l2*s23*(q(5)+q(6))),   s1*q(4)*(l1*s2+l2*s23)-c1*(l1*c2*q(5)+l2*c23*(q(5)+q(6))),   l2*s1*q(4)*s23-l2*c1*c23*(q(5)+q(6));
        -s1*q(4)*(l1*c2+l2*c23)+c1*(-l1*s2*q(5)-l2*s23*(q(5)+q(6))), -c1*q(4)*(l1*s2+l2*s23)-s1*(l1*c2*q(5)+l2*c23*(q(5)+q(6))), -l2*c1*q(4)*s23-l2*s1*c23*(q(5)+q(6));
        %0,0,0;
        0,                                                           l1*s2*q(5)+l2*s23*(q(5)+q(6)),                              l2*s23*(q(5)+q(6))  ];
    

%==========================================================================
%Perform forward kinematic computations

x(1,1) = l2*c1*c23 + l1*c1*c2;%x-coordinate of end-effector
%x(2,1) = q(1)+2*q(2)+q(3);%y-coordinate of end-effector
x(2,1) = l2*s1*c23 + l1*s1*c2;%y-coordinate of end-effector
x(3,1) = -l1*s2 - l2*s23;%z-coordinate of the end-effector

%% ------------ Projection matrix ----------------------------------
% Z direction
% A = [0, (-l1*c2-l2*c23), -l2*c23];
% drv_A = [0, (l1*s2*q(5)+l2*s23*(q(5)+q(6))), (l2*s23)*(q(5)+q(6))];
% 
% P = eye(3) - pinv(A) * A;
% 
% C = -pinv(A) * drv_A;
%EOF
%==========================================================================
   
% R23=[c3 -s3 0;s3 c3 0;0 0 1];
% R3s=[1 0 0;0 1 0;0 0 1];
% R0s=R01*R12*R23*R3s;
% w0s=R0s.ws=R0s.[-s23*q(4);-c23*q(4);q(5)+q(6)];
% w0s=[-s1*(q(5)+q(6));c1*(q(5)+q(6));q(4)];

%Det(J)=-c1^2*c2*c23*s2*l1^2*l2-c2*c23*s1^2*s2*l1^2*l2+c1^2*c2^2*s23*l1^2*l2+c2^2*s1^2*s23*l1^2*l2-c1^2*c23^2*s2*l1*l2^2-c23^2*s1^2*s2*l1*l2^2+c1^2*c2*c23*s23*l1*l2^2+c2*c23*s1^2*s23*l1*l2^2;
%simplify[Det(J)]=c2*l1^2*l2*(c2*s23-s2*c23)+c23*l1*l2^2(c2+s2*c23*(c1^2-s1^2));


% Jw =[-c1*(q(5)+q(6)), 0, 0;-s1*(q(5)+q(6)), 0, 0;0 , 0, 0];
% Jtot=[-s1*(l1*c2+l2*c23),  -c1*(l1*s2+l2*s23),   -l2*c1*s23;
%     c1*(l1*c2+l2*c23),-s1*(l1*s2+l2*s23),   -l2*s1*s23;
%     0,                -(l1*c2+l2*c23),      -l2*c23;
%    -c1*(q(5)+q(6)), 0, 0;-s1*(q(5)+q(6)), 0, 0;0 , 0, 0];
% dJ= ;

% [U,S,V] = svd(J);
% [det(J),diag(S).'];
% CN = cond(J);