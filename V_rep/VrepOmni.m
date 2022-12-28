
clc;clear all;close all

%% --------- VREP Setup ----------------------
% Specify paths
path = 'C:\Program Files\V-REP3\V-REP_PRO_EDU';
libpath = { fullfile(path, 'programming', 'remoteApi')
    fullfile(path, 'programming', 'remoteApiBindings', 'matlab', 'matlab')
    fullfile(path, 'programming', 'remoteApiBindings', 'lib', 'lib')
    }; addpath( libpath{:} );

disp('Program started');
vrep = remApi('remoteApi','extApi.h');
vrep.simxFinish(-1);
client = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if (client>-1)
    disp('Connected to remote API server');
end

% Constants
vrep_steptime = 50; % from vrep [ms]
oneshot_wait = vrep.simx_opmode_oneshot_wait;
oneshot = vrep.simx_opmode_oneshot;
stream = vrep.simx_opmode_streaming;
buffer = vrep.simx_opmode_buffer;

% % Get joint handles
% [~, joint{1}] = vrep.simxGetObjectHandle(client,'LBR_iiwa_7_R800_joint2', oneshot_wait);
% [~, joint{2}] = vrep.simxGetObjectHandle(client,'LBR_iiwa_7_R800_joint4', oneshot_wait);
% [~, joint{3}] = vrep.simxGetObjectHandle(client,'LBR_iiwa_7_R800_joint6', oneshot_wait);

% Get joint handles
[~, joint{1}] = vrep.simxGetObjectHandle(client,'waist', oneshot_wait);
[~, joint{2}] = vrep.simxGetObjectHandle(client,'shoulder', oneshot_wait);
[~, joint{3}] = vrep.simxGetObjectHandle(client,'elbow', oneshot_wait);

%% --------- Time Definition ----------------------
% Options
t_end = 20; % end time
dt = vrep_steptime/1000; % step time [s]

% Initial positions
% q0 = [[0, 0, 90]*pi/180 ...   % joint positions
% q0 = [[0, -60 , 120]*pi/180 ...   % joint positions
% q0 = [[0, 45 , 0]*pi/180 ...   % joint positions
% q0 = [[0, 30 , 0]*pi/180 ...   % joint positions
q0 = [[0, 60 , -30]*pi/180 ...   % joint positions
      [0, 0, 0]*pi/180];       % joint speeds
  
for ii = 1:length(joint);
    vrep.simxSetJointPosition(client,joint{ii},q0(ii),oneshot_wait);
end

%% --------- Simulation ----------------------
t = 0:dt:t_end;
q = zeros(length(t),length(q0)); %joint positions and vel
u = zeros(length(t),3); %control input
% x0(1:3)=[0.135 0 0]; %lenght(m)
% x0(1:3)=[0.270*sqrt(2) 0 0]; %lenght(m)
% x0(1:3)=[0.135*sqrt(2) 0 0]; %lenght(m)
% x0(1:3)=[0.135*sqrt(3) 0 0]; %lenght(m)
x0(1:3)=[0.135*0.5*(1+sqrt(3)) 0 0]; %lenght(m)

% qd = [0.1*sin(t)',45*(pi/180)*ones(length(t),1) 0*(pi/180)*ones(length(t),1)];
% dqd = [0.1*cos(t)',zeros(length(t),2)];
% ddqd = [-0.1*sin(t)',zeros(length(t),2)];
Nt=401;
ddxd = zeros(Nt,3);
dxd = zeros(Nt,3);
%xd = [(x0(1)-0.005*t)', zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.
%xd = [(x0(1) - 0.05*t), x0(2)*ones(2000,1), zeros(2000,1)]; %const. velocity motion in x-dir.
%xd = [(x0(1)+ 0.00675*t)', zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.
% xd = [(x0(1)- 0.0135*(1-sqrt(2))*t)', zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.
% xd = [(x0(1)+ 0.0135*(2-sqrt(2))*t)', zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.
% xd = [(x0(1)+ 0.135*(2-sqrt(3))*t)', zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.
xd = [(x0(1)+ 0.135*((3-(sqrt(3)))/2)*t)', zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.

% Initial values
q(1,:) = q0;

% Simulation with Vrep
vrep.simxSynchronous(client,true);
vrep.simxStartSimulation(client,oneshot);
fprintf('\n-- Simulation started \n')

% Vrep state
for jj = 1:3
    % Joint angle
    [~, q(1,jj)] = vrep.simxGetJointPosition(client, joint{jj},oneshot_wait);
    % Constraint Force
    [~, Fq(1,jj)] = vrep.simxGetJointForce(client, joint{jj},oneshot_wait);
end

for ii = 1:length(t)-1           
    % Control
    
     m2 = 0.035; m3 = 0.1;m1 = 0.35;


    MProp = [m1, m2, m3]; %define mass
    
     l1 = 0.135; l2 = 0.135; l3 = 0;

    LProp = [l1, l2, l3]; %define link lengths
    
%   [M,H,G,J,dJ,x,Js]  = RRR_Planar_KinDyn(q(ii,:),MProp,LProp);
    [M,H,J,dJ,x,Js,CN]  = RRR_Planar_KinDyn(q(ii,:));

    % Joint Space 
    % Motion Control
    %Gd = 10*eye(3); Gp =10*eye(3);
    %dE = dqd(ii,:)' - q(ii,4:6)';
    %E = qd(ii,:)' - q(ii,1:3)';
  
    % Joint Space Motion Control
    %fj = inv(M)*(M*(ddqd(ii,:)' + Gd*dE + Gp*E)+C+G-(G+C));

    %u(ii,:) = fj;
    
td = t; %time vector used for interpolations
%Interpolate the desired end-effector position, velocity and acceleration at time t
xd = interp1(td,xd,t);
xd_TS = xd([2,3]).'; %task space desired motion
xd_NS = xd(1).'; %null-space desired motion

x_TS = x([2,3]); %y-axis translation and z-axis rotation are the controlled coordinates
x_NS = x([1]); %x is the singular direction controlled in the null-space

%Task space desired velocity and acceleration are zero
dxd_TS = zeros(2,1);
ddxd_TS = zeros(2,1);

dx = J*q([4:6]).'; %get current end-effector speed

dx_TS = dx([2,3]); %current task space velocities
dx_NS = dx(1); %current null-space velocities

    %Task space force calculation
Kvm = 5*eye(2); %gain for velocity control
Kpm = 25*eye(2); %gain for position control

e_TS = xd_TS - x_TS; %error in position
de_TS = dxd_TS - dx_TS; %error in velocity
  
%Task-space (operational) force per unit mass (e.g., fm_star) 
% Fm_TS = ddxd_TS + Kvm*(de_TS).' + Kpm*(e_TS).';
Fm_TS = ddxd_TS + Kvm*(de_TS) + Kpm*(e_TS);

%Null-space desired velocity and acceleration 
% dxd_NS = 0.00675;
% dxd_NS = -0.0135*(1-sqrt(2));
% dxd_NS = 0.0135*(2-sqrt(2));
% dxd_NS = 0.0135*(2-sqrt(3));
dxd_NS = 0.0135*((3-(sqrt(3)))/2);
ddxd_NS = 0;

%Null-space force calculation (for controlling singular DOF)
e_NS = xd_NS - x_NS; %error in position
de_NS = dxd_NS - dx_NS; %error in velocity

Fm_NS = ddxd_NS + 3*de_NS + 1.5*e_NS;

Gamma_0 = J*[Fm_NS;0;0]; %get null-space joint torques

JsR= Js([2,3],:); %the singularity Jacobian

Lambda_r = inv(JsR*inv(M)*(JsR.')); %the redundant pseudo kinect energy matrix
Jb = inv(M)*(JsR.')*Lambda_r; %dynamically consistent inverse of the singularity jacobian 

%Total torque is sum of task-space and null-space torques

%Gammaj = JsR.'*Lambda_r*Fm_TS + (eye(3)- JsR.'*Jb.')*Gamma_0 +(H);
%Gammaj = JsR.'*Lambda_r*Fm_TS +(J.'*(0.1*eye(3)*(xd'-x)+0.05*eye(3)*(0-dx))+(H));

Gammaj = JsR'*Lambda_r*Fm_TS + J.'*(200*eye(3)*([xd_NS;0;0] - [x_NS;0;0])+50*eye(3)*(0-dx))+(H);

qdd = inv(M)*((Gammaj - (H)));

u(ii,:) = Gammaj
%u(ii,:) =qdd;

 
 
    vel = 9999;
    for jj = 1:3
        if( u(ii,jj) >0.0 )
            vel1 = vel;
        elseif (u(ii,jj) <0.0)
            vel1 = -vel;
        else
            vel1 = 0;
        end
        
        vrep.simxSetJointTargetVelocity(client,joint{jj},vel1,oneshot);
        vrep.simxSetJointForce(client,joint{jj},abs(u(ii,jj)),oneshot);
  
    end
    
    % Vrep commands
    signal = vrep.simxPackFloats(u(ii,:));
    vrep.simxSetStringSignal(client,'u',signal,oneshot_wait);    
   
    %Simulation step
    vrep.simxSynchronousTrigger(client);
    % Vrep state
    for jj = 1:3
        % Joint angle
        [~, q(ii+1,jj)] = vrep.simxGetJointPosition(client, joint{jj},oneshot_wait);
        % Joint velocity
        [~, q(ii+1,3+jj)] = vrep.simxGetObjectFloatParameter(client, joint{jj}, 2012, oneshot_wait);
        % End Effector Position

    end
    
end

vrep.simxPauseSimulation(client,oneshot);

x=zeros(Nt,3);
for i=1:Nt
x(i,1) = l2*cos(q(i,1))*cos(q(i,2)+q(i,3)) + l1*cos(q(i,1))*cos(q(i,2));%x-coordinate of end-effector
x(i,2) = l2*sin(q(i,1))*cos(q(i,2)+q(i,3)) + l1*sin(q(i,1))*cos(q(i,2));%y-coordinate of end-effector
%x(i,2) = q(i,1)+2*q(i,2)+q(i,3);%y-coordinate of end-effector
x(i,3) = -l1*sin(q(i,2))-l2*sin(q(i,2)+q(i,3));%z-coordinate of the end-effector
end
%% --------- Plots --------------------------
% subplot(3,1,1);plot(t,q(:,1));hold on;plot(t,qd(:,1),'--r');xlabel('Time [s]');ylabel('\theta_1 [rad]');grid on;
% title('Vrep Data','fontweight','bold');
% subplot(3,1,2);plot(t,q(:,2));hold on;plot(t,qd(:,2),'--r');xlabel('Time [s]');ylabel('\theta_2 [rad]');grid on;
% subplot(3,1,3);plot(t,q(:,3));hold on;plot(t,qd(:,3),'--r');xlabel('Time [s]');ylabel('\theta_3 [rad]');grid on;

subplot(3,1,1);plot(t,x(:,1));hold on;plot(t,xd(:,1),'--r');xlabel('Time [s]');ylabel('\x_1 [mm]');grid on;
title('Vrep Data','fontweight','bold');
subplot(3,1,2);plot(t,x(:,2));hold on;plot(t,xd(:,2),'--r');xlabel('Time [s]');ylabel('\x_2 [mm]');grid on;
subplot(3,1,3);plot(t,x(:,3));hold on;plot(t,xd(:,3),'--r');xlabel('Time [s]');ylabel('\x_3 [mm]');grid on;

%% --------- VREP Close  ---------------

if client >= -1
    disp('Closing vrep connection')
    vrep.simxFinish(client);
    pause(0.5);
end


