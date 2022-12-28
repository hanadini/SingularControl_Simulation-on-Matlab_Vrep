%Control of motion at a singularity using null-space torques
%
%For the details, see:
%K.-S. Chang and O. Khatib "Manipulator Control at Kinematic Singularities:
%A Dynamically Consistent Strategy"(1995).

%==========================================================================
%INITIALIZE

%Initialize workspace
clc;clear all;close all; %initialize

%Define time
dt = 0.01; %sampling time
Nt = 2000;%1e3 %total number of samples
t = [0:dt:(Nt-1)*dt]'; %time vector
tsim = t;

%==========================================================================
%DEFINE ROBOT

%Physical properties of the robot
m2 = 0.035;  m3 = 0.1;m1 = 0.35; MProp = [m1, m2, m3]; %define mass
l1 = 0.135; l2 = 0.135; l3 = 0; LProp = [l1, l2, l3]; %define link lengths

% q0 = [0;0;0]; %initial joint angles
q0 = [0;-pi/3;2*pi/3]; %initial joint angles
dq0 = [0;0;0]; %initial joint velocities

%Get current kinamtic and dynamic properties
% [M,h,Jv,dJv,x0,Js,CN] = RRR_Planar_KinDyn([q0;dq0]);
% clear M h Jv dJv Js CN ; %we just need the current end-effector position

[M,h,Jv,dJv,x0,Js] = RRR_Planar_KinDyn([q0;dq0]);  % 6*1 vector
clear M h Jv dJv Js; %we just need the current end-effector position

%Define desired JS motion ( Joint Space )
td = t; %time vector used for interpolations

%Define desired OS motion (Operational Space )
td = t; %time vector used for interpolations

ddxd = zeros(Nt,3);
dxd = zeros(Nt,3);

%xd = [(x0(1) + 0.00428*t), zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir. in 20 sec
xd = [(x0(1) + 0.00675*t), zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir. in 20 sec
%xd = [(x0(1)+0.009*t), zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir. in 15 sec
% xd = [(x0(1) - 0.05*t), x0(2)*ones(Nt,1), zeros(Nt,1)]; %const. velocity motion in x-dir.
% xd = [(x0(1)+0.135*1.5385*sin(t)), zeros(Nt,1), x0(3)*ones(Nt,1)]; %const. velocity motion in x-dir.
%==========================================================================
%SETUP DYNAMICS INTEGRATOR

y0 = [q0;dq0]; %initial state vector

Nstates = length(y0); %get number of states
y = zeros(Nstates,Nt); %initialize state output vector

Gamma = zeros(3,Nt); %initialize joint torque vector

%Method coefficients -- Butcher's tableau
C = [1/5; 3/10; 4/5; 8/9; 1];

A = [1/5,          0,           0,            0,         0
    3/40,         9/40,        0,            0,         0
    44/45        -56/15,       32/9,         0,         0
    19372/6561,  -25360/2187,  64448/6561,  -212/729,   0
    9017/3168,   -355/33,      46732/5247,   49/176,   -5103/18656];

B = [35/384, 0, 500/1113, 125/192, -2187/6784, 11/84];

%More convenient storage
A = A.';B = B(:);

Nstages = length(B);
K = zeros(Nstates,Nstages);

y(:,1) = y0; %set first step @ t = 0 to q0

%==========================================================================
%SIMULATION (using an ODE5 solver)
% 
% for indt = 2:Nt, %loop over the time vector
%     
%     t_past = t(indt-1); %get previous time
%     y_past = y(:,indt-1); %get previous states
%     
%     %General explicit Runge-Kutta framework
%     
%     [K(:,1), Gamma(:,indt)] = SingControl_RRR(t_past, y_past, MProp, LProp, td, ddxd, dxd, xd);
%     
%     for indstage = 2:Nstages
%     
%         tstage = t_past + C(indstage-1)*dt;
%         ystage = y_past + K(:,1:indstage-1) * (dt * A(1:indstage-1,indstage-1));    
%         K(:,indstage) = SingControl_RRR(tstage, ystage, MProp, LProp, td, ddxd, dxd, xd);
%     
%     end
%     
%     y(:,indt) = y_past + K * (dt * B);
%     
%     if mod(t(indt),0.1) == 0, display(['Current simulation time: ', num2str(t(indt)), ' s.']); end
%     
% end %end of simulation
% 
% %Post-process the results
% q = y.';
% Gamma = Gamma.';
%==========================================================================
%SIMULATION (using an ODE5 solver)

for indt = 2:Nt, %loop over the time vector
    
    t_past = t(indt-1); %get previous time
    y_past = y(:,indt-1); %get previous states
    
    %General explicit Runge-Kutta framework
    
    [K(:,1), Gamma(:,indt)] = SingControl_RRR(t_past, y_past, MProp, LProp, td, ddxd, dxd, xd);
    
    for indstage = 2:Nstages
    
        tstage = t_past + C(indstage-1)*dt;
        ystage = y_past + K(:,1:indstage-1) * (dt * A(1:indstage-1,indstage-1));    
        [K(:,indstage),dummy,CN(indt,1)] = SingControl_RRR(tstage, ystage, MProp, LProp, td, ddxd, dxd, xd);
    
    end
    
    y(:,indt) = y_past + K * (dt * B);
    
    if mod(t(indt),0.1) == 0, display(['Current simulation time: ', num2str(t(indt)), ' s.']); end
    
end %end of simulation

%Post-process the results
q = y.';
Gamma = Gamma.';

%==========================================================================
%PLOTS
nfig = 1;

%Plot joint angles
figure(nfig)
ax = gca;set(ax,'FontName', 'Times New Roman', 'FontSize', 14);hold on;box on;grid on;
plot(tsim,q(:,[1:3]),'LineWidth',2);
ylabel('Joint Angles ( {\it q_i} )');
xlabel('Time [s]');
%ylim([1 6])
hold off;
nfig = nfig + 1;

%Plot joint velocities
figure(nfig)
ax = gca;set(ax,'FontName', 'Times New Roman', 'FontSize', 14);hold on;box on;grid on;
plot(tsim,q(:,[4:6]),'LineWidth',2);
ylabel('Joint Velocities ( {\it dq_i / dt} )')
xlabel('Time [s]');
%ylim([0 5])
hold off;
nfig = nfig + 1;

%Plot end-effector position
x = RRR_Planar_ForKin(q,LProp,Nt);

% % [Mq,hq,J,dJ,x,Js] = RRR_Planar_KinDyn(q);
% % clear Mq hq J dJ Js;

% x =zeros(Nt,3);
% for i=1:Nt
%     x(i,1) = l2*cos(q(1))*cos(q(2)+q(3)) + l1*cos(q(1))*cos(q(2));%x-coordinate of end-effector
%     x(i,2) = l2*sin(q(1))*cos(q(2)+q(3)) + l1*sin(q(1))*cos(q(2));%y-coordinate of end-effector
%     x(i,3) = -l1*sin(q(2)) - l2*sin(q(2)+q(3));%z-coordinate of the end-effector
% %     x(i,2) = -l1*sin(q(2)) - l2*sin(q(2)+q(3));%z-coordinate of the end-effector
% %     x(i,3) = l2*sin(q(1))*cos(q(2)+q(3)) + l1*sin(q(1))*cos(q(2));%y-coordinate of end-effector
%     
% end


figure(nfig)
ax = gca;set(ax,'FontName', 'Times New Roman', 'FontSize', 14);hold on;box on;grid on;
plot(tsim,x,'LineWidth',2);
plot(tsim,xd,'--r','LineWidth',2);
ylabel('End-effector position ( {\itx_i} ) [m]')
xlabel('Time [s]');
hold off;
nfig = nfig + 1;

% Plot Condition Number
figure(nfig)
plot (x (:,1),CN);xlabel ('x position');ylabel ('condition number');
hold off;
nfig = nfig + 1;

%Animation
figure(nfig)

%Animation

% writerObj = VideoWriter('ElbowNWristSingularity');
% open(writerObj);

sf = 100;

% x_pos = [l1*cos(q(:,1)), l1*cos(q(:,1)) + l2*cos(q(:,1)+q(:,2)), l1*cos(q(:,1)) + l2*cos(q(:,1)+q(:,2)) + l3*cos(q(:,1)+q(:,2)+q(:,3))];
% y_pos = [l1*sin(q(:,1)), l1*sin(q(:,1)) + l2*sin(q(:,1)+q(:,2)), l1*sin(q(:,1)) + l2*sin(q(:,1)+q(:,2)) + l3*sin(q(:,1)+q(:,2)+q(:,3))];

% x_pos = [0,cos(q(:,1))*cos(q(:,2))*l1,l2*cos(q(:,1))*cos(q(:,2)+q(:,3)) + l1*cos(q(:,1))*cos(q(:,2))];%x-coordinate of end-effector
% y_pos = [0,sin(q(:,1))*cos(q(:,2))*l1,l2*sin(q(:,1))*cos(q(:,2)+q(:,3)) + l1*sin(q(:,1))*cos(q(:,2))];%y-coordinate of end-effector
% % z_pos = [0,-sin(q(:,2))*l1,-l1*sin(q(:,2))-l2*sin(q(:,2)+q(:,3))];%z-coordinate of end-effector

x_pos = zeros(Nt,3);
y_pos = zeros(Nt,3);
z_pos = zeros(Nt,3);
for i=1:Nt 
x_pos(i,:) = [0,cos(q(i,1))*cos(q(i,2))*l1,l2*cos(q(i,1))*cos(q(i,2)+q(i,3)) + l1*cos(q(i,1))*cos(q(i,2))];%x-coordinate of end-effector
%y_pos(i,:) = [0,sin(q(i,1))*cos(q(i,2))*l1,l2*sin(q(i,1))*cos(q(i,2)+q(i,3)) + l1*sin(q(i,1))*cos(q(i,2))];%y-coordinate of end-effector
y_pos(i,:)= [0,0,0];
z_pos(i,:)= [0,-sin(q(i,2))*l1,-l1*sin(q(i,2))-l2*sin(q(i,2)+q(i,3))];%z-coordinate of end-effector
%z_pos(i,:)= [0,0,0];%z-coordinate of end-effector
end


h = plot(0,0,'k^','LineWidth',2);
ax = gca;set(ax,'FontName', 'Times New Roman', 'FontSize', 14);
hold on;grid on;box on;

% h = plot(x_pos(1,:),y_pos(1,:),'ko','LineWidth',2);
% j1 = plot([0, x_pos(1,1)],[0, y_pos(1,1)],'LineWidth',2);
% j2 = plot([x_pos(1,1), x_pos(1,2)],[y_pos(1,1), y_pos(1,2)],'LineWidth',2);
% j3 = plot([x_pos(1,2), x_pos(1,3)],[y_pos(1,2), y_pos(1,3)],'LineWidth',2);
% j4 = plot(x_pos(1,3),y_pos(1,3),'.r');

% h = plot(x_pos(1,:),y_pos(1,:),'ko','LineWidth',2);
% j1 = plot([0, x_pos(1,1)],[0, y_pos(1,1)],'LineWidth',2);
% j2 = plot([x_pos(1,1), x_pos(1,2)],[y_pos(1,1), y_pos(1,2)],'LineWidth',2);
% j3 = plot([x_pos(1,2), x_pos(1,3)],[y_pos(1,2), y_pos(1,3)],'LineWidth',2);
% j4 = plot(x_pos(1,3),y_pos(1,3),'.r');

h = plot(x_pos(1,:),z_pos(1,:),'ko','LineWidth',2);    %(x_pos(:,1)and z_pos(:,1) transposed.
j1 = plot([0, x_pos(1,1)],[0, z_pos(1,1)],'LineWidth',2);
j2 = plot([x_pos(1,1), x_pos(1,2)],[z_pos(1,1), z_pos(1,2)],'LineWidth',2);
j3 = plot([x_pos(1,2), x_pos(1,3)],[z_pos(1,2), z_pos(1,3)],'LineWidth',2);
j4 = plot(x_pos(1,3),z_pos(1,3),'.r');

hold off;
axis([-0.3 0.3 -0.3 0.3]);

% frame = getframe;
% writeVideo(writerObj,frame);

for indt = 2:Nt
%     set(j1(1),'xdata',[0, x_pos(indt,1)],'ydata',[0, y_pos(indt,1)],'Color','b');
%     set(j2(1),'xdata',x_pos(indt,[1:2]),'ydata',y_pos(indt,[1:2]),'Color','b');
%     set(j3(1),'xdata',x_pos(indt,[2:3]),'ydata',y_pos(indt,[2:3]));
%     set(j4(1),'xdata',x_pos(1:indt,3),'ydata',y_pos(1:indt,3));
%     set(h(1),'xdata',x_pos(indt,:),'ydata',y_pos(indt,:));
    set(j1(1),'xdata',[0,x_pos(indt,1)],'zdata',[0,z_pos(indt,1)],'Color','b');
    set(j2(1),'xdata',x_pos(indt,[1:2]),'zdata',z_pos(indt,[1:2]),'Color','b');
    set(j3(1),'xdata',x_pos(indt,[2:3]),'zdata',z_pos(indt,[2:3]));
    set(j4(1),'xdata',x_pos(1:indt,3),'zdata',z_pos(1:indt,3));
    set(h(1),'xdata',x_pos(indt,:),'zdata',z_pos(indt,:));
    refreshdata(j1,'caller');
    refreshdata(j2,'caller');
    refreshdata(j3,'caller');
    refreshdata(j4,'caller');
    refreshdata(h,'caller');
    
% drawnow expose;
    pause(0.001);
    
%     frame = getframe;
%     writeVideo(writerObj,frame);
end
nfig = nfig + 1;

% close(writerObj);
%EOF