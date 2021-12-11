clear;
clc;
close all; 
%% Helper Functions
position = @(transform) transform(1:3,4);
rotation = @(transform) transform(1:3,1:3);
%% Forward Kinematics (4 Link Robot)
robot = generateRobot;
show(robot,[-pi/4,pi/4,pi/4,0,0,0]');
%% Load params
param = gaitParameters(robot);
%% Generate Trajectory
isHalfStep = true;
param.swingFoot = 'right_lower_leg';
param.supportFoot = 'left_lower_leg';
trajectory1 = [0;0;0]
trajectory2 = [0.2;0;0.2]
trajectory3 = [0.1;0;0.1]
trajectory4 = [0.05;0;0.05]
%% Inverse Kinematics
[opt_joint_angles_1] = inverseKinematics(robot,trajectory1,param);
[opt_joint_angles_2] = inverseKinematics(robot,trajectory2,param);
[opt_joint_angles_3] = inverseKinematics(robot,trajectory3,param);
[opt_joint_angles_4] = inverseKinematics(robot,trajectory4,param);

figure;
subplot(2,2,1)
p1 = show(robot,opt_joint_angles_1);
ax = gca;
ax.View = [180 0];
hold on;
Hsp = getTransform(robot,opt_joint_angles_1,'left_lower_leg')
point = Hsp*[trajectory1;1]
p2 = plot3(point(1),point(2),point(3),'*','LineWidth',4,'color','red')
legend(p2([1]),{'Desired Swing Foot Position'})
subplot(2,2,2)
p1 = show(robot,opt_joint_angles_2);
ax = gca;
ax.View = [180 0];
hold on;
Hsp = getTransform(robot,opt_joint_angles_2,'left_lower_leg')
point = Hsp*[trajectory2;1]
p2 = plot3(point(1),point(2),point(3),'*','LineWidth',4,'color','red')
legend(p2([1]),{'Desired Swing Foot Position'})

subplot(2,2,3)
p1 = show(robot,opt_joint_angles_3);
ax = gca;
ax.View = [180 0];
hold on;
Hsp = getTransform(robot,opt_joint_angles_3,'left_lower_leg')
point = Hsp*[trajectory3;1]
p2 = plot3(point(1),point(2),point(3),'*','LineWidth',4,'color','red')
legend(p2([1]),{'Desired Swing Foot Position'})

subplot(2,2,4)
p1 = show(robot,opt_joint_angles_4);
ax = gca;
ax.View = [180 0];
hold on;
Hsp = getTransform(robot,opt_joint_angles_4,'left_lower_leg')
point = Hsp*[trajectory4;1]
p2 = plot3(point(1),point(2),point(3),'*','LineWidth',4,'color','red')

legend(p2([1]),{'Desired Swing Foot Position'})