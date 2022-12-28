function [x] = RRR_Planar_ForKin(q,LProp,Nt)

l1 = LProp(1);l2 = LProp(2); l3 = LProp(3); %get link lengths

%==========================================================================
%Define some trigonometric functions

c1 = cos(q(:,1));         c2 = cos(q(:,2));         c3 = cos(q(:,3));
s1 = sin(q(:,1));         s2 = sin(q(:,2));         s3 = sin(q(:,3));
c12 = cos(q(:,1)+q(:,2));   s12 = sin(q(:,1)+q(:,2));
c23 = cos(q(:,2)+q(:,3));   s23 = sin(q(:,2)+q(:,3));
c123 = cos(q(:,1)+q(:,2)+q(:,3));
s123 = sin(q(:,1)+q(:,2)+q(:,3));

c2_2 = cos(2*q(:,2));     c2_23 = cos(2*(q(:,2)+q(:,3))); c22_3 = cos(2*q(:,2)+q(:,3));
s2_2 = sin(2*q(:,2));     s2_23 = sin(2*(q(:,2)+q(:,3))); s22_3 = sin(2*q(:,2)+q(:,3)); 

%==========================================================================
%Perform forward kinematic computations

% x = zeros(Nt,3);
% for i=1:Nt
% x(i,1) = l2*c1(i,1)*c23(i,1) + l1*c1(i,1)*c2(i,1);%x-coordinate of end-effector
% x(i,2) = q(i,1)+2*q(i,2)+q(i,3);%y-coordinate of end-effector
% x(i,3) = -l1*s2(i,1) - l2*s23(i,1);%z-coordinate of the end-effector
% end

x = zeros(Nt,3);
for i=1:Nt
x(i,1) = l2*c1(i,1)*c23(i,1) + l1*c1(i,1)*c2(i,1);%x-coordinate of end-effector
x(i,2) = l2*s1(i,1)*c23(i,1) + l1*s1(i,1)*c2(i,1);%y-coordinate of end-effector
x(i,3) = -l1*s2(i,1) - l2*s23(i,1);%z-coordinate of the end-effector(z-rotation of the end-effector)
end

%==========================================================================
% x = zeros(2000,3);
% for i=1:2000
% x(i,1) =cos(q(i,1))*cos(q(i,2))*l1+(cos(q(i,1))*cos(q(i,2))*cos(q(i,3))-cos(q(i,1))*sin(q(i,2))*sin(q(i,3)))*l3;
% x(i,2) =cos(q(i,2))*sin(q(i,1))*l1+(cos(q(i,2))*cos(q(i,3))*sin(q(i,1))-sin(q(i,1))*sin(q(i,2))*sin(q(i,3)))*l2;
% x(i,3) =-sin(q(i,2))*l1+(-cos(q(i,3))*sin(q(i,2))-cos(q(i,2))*sin(q(i,3)))*l3;
% end

% x(:,1) = l2*cos(q(:,1))*cos(q(:,2)+q(:,3))+ l1*cos(q(:,1))*cos(q(:,2));%x-coordinate of end-effector
% x(:,2) = l2*sin(q(:,1))*cos(q(:,2)+q(:,3))+ l1*sin(q(:,1))*cos(q(:,2));%y-coordinate of end-effector
% x(:,3) = -l1*sin(q(:,2)) - l2*sin(q(:,2)+q(:,3));%z-coordinate of the end-effector%

% x(:,1) = l2*c1*c23+ l1*c1*c2;%x-coordinate of end-effector
% x(:,2) = l2*s1*c23 + l1*s1*c2;%y-coordinate of end-effector
% x(:,3) = -l1*s2 - l2*s23;%z-coordinate of the end-effector% 

%EOF