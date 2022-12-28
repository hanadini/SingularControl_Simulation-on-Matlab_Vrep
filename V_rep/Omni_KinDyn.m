function [Mq,hq,J,dJ,P,C,x] = Omni_KinDyn(q,I1,I2,I3)

g = 9.81;

% Physical properties of manipulator
m1 = 0.35; m2 = 0.035; m3 = 0.1; 
l1 = 0.135; l2 = 0.135; l3 = 0;

% joint angles correction
q(3) = q(3)-(pi/2);

%% ----------------- Define trigonometric functions ------------------------
c1 = cos(q(1));         c2 = cos(q(2));             c3 = cos(q(3));    
s1 = sin(q(1));         s2 = sin(q(2));             s3 = sin(q(3));
c12 = cos(q(1)+q(2));   c13 = cos(q(1)+q(3));       c23 = cos(q(2)+q(3)); 
s12 = sin(q(1)+q(2));   s13 = sin(q(1)+q(3));       s23 = sin(q(2)+q(3));
c2_2 = cos(2*q(2));     c2_23 = cos(2*(q(2)+q(3))); c22_3 = cos(2*q(2)+q(3));
s2_2 = sin(2*q(2));     s2_23 = sin(2*(q(2)+q(3))); s22_3 = sin(2*q(2)+q(3)); 


%% ------------ Compute equation of motion matrices in joint space ----------
Ixx1 = 6e-3;Iyy1 = 6e-3;        Izz1 = 6e-3;
Ixx2 = 0;   Iyy2 = 2.126e-4;    Izz2 = 2.126e-4;
Ixx3 = 0;   Iyy3 = 2.126e-4;    Izz3 = 2.126e-4;

% Ixx1 = I1(1,1);       Iyy1 = I1(2,2);    Izz1 = I1(3,3);
% Ixx2 = I2(1,1);       Iyy2 = I2(2,2);    Izz2 = I1(3,3);
% Ixx3 = I3(1,1);       Iyy3 = I3(2,2);    Izz3 = I1(3,3);


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

h1 = (1/8)*(2*q(4)*(q(5)*(4*s2_2*(Ixx2 - Iyy2) + 4*s2_23*(Ixx3 - ...
    Iyy3) - s2_2*(l1^2)*(m2 +4*m3) - 4*s22_3*l1*l2*m3 - s2_23*(l2^2)*m3))+...
    4*s23*q(4)*q(6)*(c23*(4*Ixx3 - 4*Iyy3 +l2^2*m3)-2*l1*l2*m3*c2));


h2 = (1/8)*((q(4)^2)*(4*s2_2*(Iyy2 - Ixx2) + 4*s2_23*(Iyy3 - Ixx3) + ...
    s2_2*(l1^2)*(m2 + 4*m3) + 4*s22_3*l1*l2*m3 + s2_23*(l2^2)*m3) - ...
    4*(g*c23*l2*m3 +l1*g*c2*m2 + 2*l1*g*m3^2*c2) + 8*q(5)*q(6)*s3*l1*l2*m3 + 4*l1*l2*m3*s3*q(6)^2);


h3 = (1/4)*(-2*l2*m3*g*c23 + 2*s3*(q(5)^2)*l1*l2*m3 - s23*(q(4)^2)*(4*c23*(Ixx3-Iyy3) - ...
    -l2*m3*(2*c2*l1+c23*l2)));


hq= [h1;h2;h3];

%% ------------ Perform Jacobian computations --------------------
J = [-s1*(l1*c2+l2*c23),  -c1*(l1*s2+l2*s23),   -l2*c1*s23;
        c1*(l1*c2+l2*c23),-s1*(l1*s2+l2*s23),   -l2*s1*s23;
        0,                -(l1*c2+l2*c23),      -l2*c23];

dJ = [-c1*q(4)*(l1*c2+l2*c23)-s1*(-l1*s2*q(5)-l2*s23*(q(5)+q(6))),   s1*q(4)*(l1*s2+l2*s23)-c1*(l1*c2*q(5)+l2*c23*(q(5)+q(6))),   l2*s1*q(4)*s23-l2*c1*c23*(q(5)+q(6));
        -s1*q(4)*(l1*c2+l2*c23)+c1*(-l1*s2*q(5)-l2*s23*(q(5)+q(6))), -c1*q(4)*(l1*s2+l2*s23)-s1*(l1*c2*q(5)+l2*c23*(q(5)+q(6))), -l2*c1*q(4)*s23-l2*s1*c23*(q(5)+q(6));
        0,                                                           l1*s2*q(5)+l2*s23*(q(5)+q(6)),                              l2*s23*(q(5)+q(6))  ];

%% ------------ Perform forward kinematic computations -----------

x(1,1) = l2*c1*c23 + l1*c1*c2;%x-coordinate of end-effector (the horizontal direction the constraint equation)
x(2,1) = l2*s1*c23 + l1*s1*c2;%y-coordinate of end-effector
x(3,1) = -l1*s2 - l2*s23;%z-rotation of the end-effector
%(The vertical translational motion of end effector is limited by imposing
%the following constraint equation which was calculated with forward kinematics:-l1*s2 - l2*s23)

%% ------------ Projection matrix ----------------------------------
% Z direction
A = [0, (-l1*c2-l2*c23), -l2*c23];%the Jacobian of the constraint equation with respect to the generalized coordinates
drv_A = [0, (l1*s2*q(5)+l2*s23*(q(5)+q(6))), (l2*s23)*(q(5)+q(6))];

% X direction
% A = [-s1*(l2*c23+l1*c2),-c1*(l2*s23+l1*s2),-l2*c1*s23];
% drv_A = [-c1*q(4)*(l2*c23+l1*c2)+ s1*(l2*s23*(q(5)+q(6))+l1*s2*q(5)),...
%       s1*q(4)*(l2*s23+l1*s2)-c1*(l2*c23*(q(5)+q(6))+l1*c2*q(5)), l2*s1*q(4)*s23-l2*c1*c23*(q(5)+q(6))]; 

% A = [l1*c1*c2+l2*c1*c23 -l1*s1*s2-l2*s1*s23 -l2*s1*s23];
% drv_A = [-l1*s1*c2-l2*s1*c23 -l1*s1*c2-l2*s1*c23 -l2*s1*c23];

% Pseudoinverse by Calculation
% [m,n]=size (A);
% [U,S,V]=svd (A);
% r=rank (S);
% SR=S (1:r,1:r);
% SRc=[SR^-1 zeros (r,m-r);zeros (n-r,r) zeros (n-r,m-r)];
% A_pseu=V*SRc*U.';


P = eye(3) - pinv(A) * A;%The projection operator

C = -pinv(A) * drv_A;%the time differentiation of the projection matrix