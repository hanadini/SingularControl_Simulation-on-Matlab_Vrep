function [x] = Omni_Kin(q)

l1 = 0.135;l2 = 0.135; %link lengths

% joint angles correction
q(3) = q(3)-(pi/2);
%==========================================================================
%Define some trigonometric functions

%Define trigonometric functions
c1 = cos(q(1));         c2 = cos(q(2));             c3 = cos(q(3));    
s1 = sin(q(1));         s2 = sin(q(2));             s3 = sin(q(3));
c12 = cos(q(1)+q(2));   c13 = cos(q(1)+q(3));       c23 = cos(q(2)+q(3)); 
s12 = sin(q(1)+q(2));   s13 = sin(q(1)+q(3));       s23 = sin(q(2)+q(3));
c2_2 = cos(2*q(2));     c2_23 = cos(2*(q(2)+q(3))); c22_3 = cos(2*q(2)+q(3));
s2_2 = sin(2*q(2));     s2_23 = sin(2*(q(2)+q(3))); s22_3 = sin(2*q(2)+q(3)); 

%==========================================================================
%Perform forward kinematic computations

x(:,1) = l2*c1*c23 + l1*c1*c2;%x-coordinate of end-effector
x(:,2) = l2*s1*c23 + l1*s1*c2;%y-coordinate of end-effector
x(:,3) = -l1*s2 - l2*s23;%z-rotation of the end-effector
