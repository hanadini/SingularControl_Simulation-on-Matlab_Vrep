# Simulation_Matlab_Vrep
## Singular Control of Phantom Omni Manipulator

###Introduction
Singularities in manipulators are quite complex conditions where the end effector loses its one or more degrees of freedom along or about the singular direction(s). The control of robot manipulator in singular regions and configurations is considered as a remarkable problem since large torques in the joints lead the robot to breakdown. Hence, these singular configurations must normally be avoided, and defining the specified workspace and trajectory is essential for appropriate use of robots. The reduction in the end effector mobility affects its behavior to perform desired tasks in work space. 
This study focused on the control of robot manipulator in singular region for performing its desired task. In this order, the dynamic consistency is used to avoid any acceleration and torque of joints which can influence the performance of the end effector. The regular control is applied to show how singularity can affect the manipulator behavior. The potential field controller and singular controller are used to lead the manipulator into the singular region. The potential field algorithm based on attractive forces drew the manipulator toward its singular configuration. The singular controller leads the manipulator into the singular boundary to follow its desired task. At the end, applying the switch control between regular control and singular control helps the manipulator to enter the singular region.
In purpose to design the controllers, it is important to observe the performance of manipulator during the simulation. MATLAB program and V-REP software are used to build the controllers and to monitor the manipulator in singular region. In order to develop the controllers, the essential definitions and the dynamic equations are introduced. Then, the kinematic and dynamic analysis is presented for the PHANTOM Omni manipulator as a RRR planar robot which used in this thesis.

Keywords: Manipulators, Singularity, Artificial Potential Fields.

### CONRIBUTION:
In this thesis the in-depth research is done on the control of the robot in the singular region with regard to the dynamically consistent. The null space motion is decoupled from the end effector motion and it is also based on potential function to generate the motion by the negative gradient of the attractive potential function.
Applying attractive force field leads the manipulator to its goal configuration and makes path planning a little more complicated. To design some objects as obstacles provides the opportunity to observe and analyze the behavior of the manipulator accurately. The manipulator passes through the repulsive potential field and avoids any collision with the obstacles to reach its final position. It helps to provide a better assumption of the potential field application in the null space motion.
Simulating in MATLAB is used in the aim of investigation into RRR planar robot manipulator’s singularities. In this simulation decoupling the robot control in operational space and null space directs to better and stable manipulator controlling which is based on dynamic consistency. Detecting the kinematic singularity of the robot and also computing properly the Jacobian matrix are the important parts in recognizing the configurations as the singular configurations. 
In the current study a broad overview has been done on joints, control motions in the joint space and force control of the manipulator in operational space. Finding the singular regions of the PHANTOM Omni robot manipulator is essential to apply the switch control between the regular and singular regions. This approach presents the method to enable the manipulator to enter the vicinity and also to go into the singular region with the reliable manner.
The control algorithm is performed in MATLAB in the aim of monitoring the behavior of RRR planar robot under the singular control and to guide the end effector through performing its desired task. The simulation is done in V-REP software according to the dynamic properties of the real robot manipulator.

End – effector motion controller diagram
 
![image](https://user-images.githubusercontent.com/54982873/209535095-89d0d8af-eed9-47be-8c39-f47f2649b492.png)

### Potential Field
Treating the configuration of the robot in potential field with the combination of attracting to the specific goal and repulsing obstacles is an approach with advantages and disadvantages. The less computing trajectory process would be considered as its advantage and the probability of getting stuck in local minima in this field and losing the path could be remarked as disadvantages of this approach. The goal position of the end-effector is considered as an attractive potential field and the objects or obstacles cause collisions with manipulator parts are known as repulsive fields. The tool or end-effector is drawn to the goal position or direction by attractive force and the joints avoid from collisions by repulsive forces which come from obstacles. If the attractive force and the repulsive forces become balanced, it means their resultant force equals zero, so no progress will be occurred in the manipulator motions. The desired result would be the end-effector motion toward the goal configuration while the manipulator parts try to avoid any collision with the objects scattered in the joint pace. The attractive force decreases while the end-effector approaches to its final configuration as a goal position, and repulsive forces increase when robot parts are near the collision.

### RR PLANAR ROBOT SIMULATION:
In this section, the potential functions are applied on 2-arm planar manipulator. The behavior which is expected from this manipulator is to avoid any collision with the obstacle surface and keep moving to get the target configuration.

The following Figure is depicted path and behavior of 2R planar robot for reaching target position, in the influence of repulsive and attractive potential field. It is clearly illustrated the repulsive forces how affect every individual joint for the purpose of avoiding the collision. The forces from the final configuration attract the joints to place in their desired positions. 

![image](https://user-images.githubusercontent.com/54982873/209535479-68dd1698-5b32-4298-8723-b7a38ab6e2f4.png)

As it can be followed from Figure below that the RR manipulator starts to move from the initial position [0, 1] to a the goal position of joints [0, 1] for first joint, and for second joint moves from [0, 2] as the initial position to a goal position [-1,1]. The obstacles are located in [2, -0.5] and [-2, -0.5] positions. As depicted, the motion of each joint is influenced by the potential forces, in the way to lead the manipulator to its desired joint configuration. 

![image](https://user-images.githubusercontent.com/54982873/209535619-64f9ac83-3822-4774-bc99-28cb97c0aeaf.png)

### PHANTOM OMNI MANIPULATOR

The PHANTOM Omni is a 6R haptic device as shown in Figure (4.1). It has three drive motors which are attached to its three first joints. These motors can only give the position of end effector which is set by the computer. The first three joints are used for positioning the end effector and the rest three joints are used for finding the orientations. The last three joints can be considered as spherical joints due to their intersection in one point. In this study the first three joints are used.
In this thesis, the PHANTOM robot is controlled in the purpose of following its desired task in the singular region. The end effector requires both motion and force control for representing its accurate task and performance in operational space.
In order to achieve the dynamic consistency in PHANTOM Omni, decoupling the null space motion from operational space motion is necessary. This can be achieved by computing the kinematic and dynamic equations and the relationship between operational forces and joint torques.  

![image](https://user-images.githubusercontent.com/54982873/209535718-8edf1cf7-8fb9-4779-9de1-a0228152902e.png)

![image](https://user-images.githubusercontent.com/54982873/209535737-2ed9cc32-3380-4813-b497-91d293fcb147.png)


###DYNAMICS 
In this study, it is important to know the dynamic of the PHANTOM Omni manipulator in the purpose of applying the regular and singular control on RRR serial manipulator. In this section, a dynamic analysis for Omni is presented based on the Newton-Euler algorithm.
In the Newton-Euler formulation, the equations describe linear motion and angular motion for each link of the manipulator. Of course, since each link is coupled to other links and between them there are action and reaction forces. So these equations of each link contain coupling forces and torques. By computing the inward and outward iterations it is possible to determine all terms of each link as velocity, acceleration, force and torque. 

### SINGULAR CONTROL OF OMNI MANIPULATOR

The robot manipulator in singular configuration behaves as a redundant robot which means that the rank of the Jacobian collapses by eliminating the row along the direction of the degenerated direction.
First of all, for applying the singular control, it is required to compute the singular Jacobian matrix of the PHANTOM Omni. This Singular Jacobian is built on the singular frame of the end effector respects to the base frame of the manipulator. It means that the singular frame could be different from the end effector frame. To be more direct, the singularity can be occurred not exactly along or about of end effector axis. Therefore, it is essential to calculate the rotation matrix of singular frame respects to the base frame. This rotation matrix is used to find the Jacobian matrix of the singular configuration.

#### Task space motion control
![image](https://user-images.githubusercontent.com/54982873/209536038-2c09a01a-f5d5-45d1-b375-ef3657d3ea11.png)

#### Singular control of manipulator
![image](https://user-images.githubusercontent.com/54982873/209536130-81acd232-4626-4080-85f2-3cd43100952b.png)

The control diagram as shown in Figure demonstrates decomposition of motion control in the joint space from the operational space motion control. This decoupling is based on achieving the dynamically consistent behavior of the manipulator. It means that the null space motion control torques should not have any alteration in the end effector function.

The null space torque was eliminated from the control algorithm in order to investigate the effect of null space control on the singular controller. From the Figure (4.6) can be obviously seen that the robot was not able to follow its desired direction. It means that the robot was unable to enter the singular region, and before the singular boundary it started to have unstable motion. Therefore, the singular control is necessary to lead the robot to its desired task along the singular direction, from the vicinity of singular boundary into the singular region.  

### SIMULATION AND RESULTS
In order to observe the performance of controllers in the vicinity of singular boundary and also in the singular region, MATLAB and V-rep (Virtual Robot Experimentation Platform) are used for simulation. V-rep as a manipulator simulator [7] can be run and communicate with the other coding language via remote API programming (Application Interface). MATLAB sends and receives data to and from V-rep as shown in Figure (5.1). 
V-rep is used for different applications such as fast algorithm development, industrial automation simulator, quick prototyping and verification and remote controlling. This simulator allows controlling simulation remotely from a real robot or another PC. It includes four physics engines (Bullet Physics, ODE, Newton and Vortex Dynamic) for fast and customizable dynamics calculation, to simulate real-world physics and object interactions. V-rep can calculate forward /inverse Kinematics of any type of mechanism. It contains powerful and realistic and accurate sensor simulation which can calculate minimum distance within the customizable detection volume.

![image](https://user-images.githubusercontent.com/54982873/209536314-24796bea-d9e8-493d-8faa-70e452ebae1a.png)

In V-REP simulation the Omni robot is imported as URDF (Universal Robot Description Format) as depicted in Figure (5.2). The behavior of this robot and the environment where is working, is very similar to a real robot. So it is preferred to investigate on the robot control near and also in singular boundary. Joint limitation of the real robot and the danger of damaging itself in singular boundary and configuration are other important reasons to simulate with V-REP in this study. 
The controllers’ algorithms are built in two different initial positions of PHANTOM Omni. The desired motion equations are changed regarding each initial position. The end effector is expected to have motion along x direction as a singular direction and to reach the singular configuration. For Omni manipulator, the purposed singular configuration is achievable when two adjacent links are stretched along each other. 


### POTENTIAL FILED CONTROL
As explained earlier, one of approaches to control the manipulator for performing its desired task would be the attractive potential field. In this part of the simulation Omni robot manipulator is controlled to achieve its desired position and motion by applying the attractive potential filed.
First the initial positions of the joints are defined and the final position as a goal position is assigned in potential filed algorithms. The initial joints angles are q0[0, 60,120] ×π/180 and the initial x0 of the end effector is computed as [0.135,0,0](m) and the desired motion of the end effector along x direction is defined as: xd = [(x0+ (0.135/20)*t), 0, z0( Nt )], where xd is desired direction along x axis and any movement along or about y axis is considered zero and z is computed regarding to the time iteration (Nt). x0 and z0 are representing the initial x and z position of the end effector. No motion and rotation should be done along or about y direction. This equation is derived from the condition that the end effector reaches its final position in t=20 sec. 
From the the applied torques on joint 1, 2 and 3 could be found, The first joint torques are equal to zero, which means that there is no torques on this joint for moving and rotating so this joint is fixed without any motion.

Figure below illustrates the motion of end effector in x, y and z direction. This figure is also shown the actual motion of end effector in three directions (x, y and z). It also can be seen that the end effector follows the desired x and y with high accuracy, but there is considerable error between actual motion and desired motion along z direction. One of the main reasons for this difference could perhaps be because of the mismatch between the real Omni robot and URDF model in V-rep. It means that the length of the links in URDF is not exactly same as the real Omni. Therefore, dynamic properties of V-rep model such as mass, inertia matrix and Center Of Mass are different from those features of computed in MATLAB. These differences cause that the torques which sent from MATLAB to V-rep were not enough to move the end effector along the desired z direction. But the torques would be adequate for the motion of the end effector along x direction to follow its desired task.

a)
![image](https://user-images.githubusercontent.com/54982873/209536645-6b7b41e5-7abf-48d5-8bed-94740959b9da.png)

b)
![image](https://user-images.githubusercontent.com/54982873/209536674-011ae79b-ac0a-4f3a-8dfe-4abcc8705ca6.png)



