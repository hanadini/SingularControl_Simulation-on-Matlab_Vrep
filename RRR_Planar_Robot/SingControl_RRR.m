function [dq, Gamma,CN]= SingControl_RRR(t, q, MProp, LProp, td, ddxd, dxd, xd)

%dq:     Vector of joint angles and velocities
%Gamma:  Vector of commanded joint torques

%==========================================================================
%COMPUTE SYSTEM MATRICES

%Compute kinematic and dynamic parameters of manipulator
% [M,h,J,dJ,x,Js,CN] = RRR_Planar_KinDyn(q,MProp,LProp);
[M,h,J,dJ,x,Js,CN] = RRR_Planar_KinDyn(q);

x_TS = x([2,3]); %y-axis translation and z-axis rotation are the controlled coordinates
x_NS = x(1); %x is the singular direction controlled in the null-space

JsR = Js([2,3],:); %the singularity Jacobian

Lambda_r = inv(JsR*inv(M)*(JsR.')); %the redundant pseudo kinect energy matrix
                                    %e.g., the redundant OS inertia matrix

Jb = inv(M)*(JsR.')*Lambda_r; %dynamically consistent inverse of the singularity jacobian 

%==========================================================================
%CONTROLLER

%Motion control

%Interpolate the desired end-effector position, velocity and acceleration at time t
xd = interp1(td,xd,t);
xd_TS = xd([2,3]).'; %task space desired motion
xd_NS = xd(1).'; %null-space desired motion

%Task space desired velocity and acceleration are zero
dxd_TS = zeros(2,1);
ddxd_TS = zeros(2,1);

%Null-space desired velocity and acceleration 
% dxd_NS = -0.05;
% dxd_NS = 0.009;
 dxd_NS = 0.00675;
 %dxd_NS = 0.00428;
 ddxd_NS = 0;

dx = J*q([4:6]); %get current end-effector speed

dx_TS = dx([2,3]); %current task space velocities
dx_NS = dx(1); %current null-space velocities

%Task space force calculation
Kvm = 0.05*eye(2); %gain for velocity control
Kpm = 0.25*eye(2); %gain for position control
% Kvm = 30*eye(2); %gain for velocity control
% Kpm = 15*eye(2); %gain for position control


e_TS = xd_TS - x_TS; %error in position
de_TS = dxd_TS - dx_TS; %error in velocity

%Task-space (operational) force per unit mass (e.g., fm_star) 
Fm_TS = ddxd_TS + Kvm*de_TS + Kpm*e_TS;

%Null-space force calculation (for controlling singular DOF)
e_NS = xd_NS - x_NS; %error in position
de_NS = dxd_NS - dx_NS; %error in velocity

%Fm_NS = ddxd_NS + .3*de_NS + .15*e_NS;
Fm_NS = ddxd_NS + .5*(xd_NS - x_NS);

Gamma_0 = J*[Fm_NS;0;0]; %get null-space joint torques

%Total torque is sum of task-space and null-space torques

%Gamma = (JsR.'*Lambda_r*Fm_TS + (eye(3)- JsR.'*Jb.')*Gamma_0 + (h));
% Gamma = [0 0 0;0 1 0;0 0 1]*(JsR.'*Lambda_r*Fm_TS + (eye(3)- JsR.'*Jb.')*Gamma_0 + (h));
Gamma = JsR.'*Lambda_r*Fm_TS +(J.'*(0.1*eye(3)*(xd'-x)+0.05*eye(3)*(0-dx))+(h));

qdd = inv(M)*((Gamma - (h))); %inverse dynamics equation

%Generate the output (joint velocities and accelerations)
dq = zeros(6,1); %initialize

dq(1) = q(4);       %dq1 = q4
dq(2) = q(5);       %dq2 = q12
dq(3) = q(6);       %dq3 = q6
dq(4) = qdd(1);     %dq4 = ddq1
dq(5) = qdd(2);     %dq5 = ddq2
dq(6) = qdd(3);     %dq6 = ddq3


%EOF