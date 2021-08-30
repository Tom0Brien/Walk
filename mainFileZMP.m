clc;
clear;
close all;
addpath('./transforms');
addpath('./ZMP');
%% Forward kinematics
robot = importrobot('NUgus.urdf');
% smimport('NUgus.urdf')
robot.DataFormat = 'column';
close all;
show(robot);
com = centerOfMass(robot,homeConfiguration(robot));
hold on;
plot3(com(1),com(2),com(3),'o');
%% Params
param = gaitParameters(robot);
%% ZMP Trajectory
[com_x, com_y, zmp_x, zmp_y] = generateCoMTraj(param);
%% Trajectory Plan Half Step
param.support_foot = 'left_foot';
param.swingFoot = 'right_foot';
is_half_step = true;
param.stepCount = 1;
foot_traj_1 = generateFootTrajectory(param,is_half_step);
param.com_x = com_x;
param.com_y = com_y;
param.zmp_x = zmp_x;
param.zmp_y = zmp_y;
rCdWw = [com_x(1:param.N_samples);com_y(1:param.N_samples);zeros(1,param.N_samples)];
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_1] = inverseKinematicsZMP(param,foot_traj_1,rCdWw);
%% Trajectory Plan Full Step
param.support_foot = 'right_foot';
param.swingFoot = 'left_foot';
is_half_step = false;
param.stepCount = 2;
param.initial_conditions = opt_joint_angles_1(:,end);
foot_traj_2 = generateFootTrajectory(param,is_half_step);
rCdWw = [com_x(param.N_samples+1:param.N_samples*2);com_y(param.N_samples+1:param.N_samples*2);zeros(1,param.N_samples)];
%% Inverse Kinematics - Full Step - support foot -> left
[opt_joint_angles_2] = inverseKinematicsZMP(param,foot_traj_2,rCdWw);
%% Trajectory Plan Full Step
param.support_foot = 'left_foot';
param.swingFoot = 'right_foot';
is_half_step = false;
param.stepCount = 3;
param.initial_conditions = opt_joint_angles_2(:,end);
foot_traj_3 = generateFootTrajectory(param,is_half_step);
rCdWw = [com_x(2*param.N_samples+1:3*param.N_samples);com_y(2*param.N_samples+1:3*param.N_samples);zeros(1,param.N_samples)];
%% Inverse Kinematics - Full Step - support foot -> left
[opt_joint_angles_3] = inverseKinematicsZMP(param,foot_traj_3,rCdWw);
%% Plot Walking
% Plot first step
figure
param.support_foot = 'left_foot';
param.swingFoot = 'right_foot';
plotWalk(opt_joint_angles_1,robot,param);
% Plot 2nd step
param.support_foot = 'right_foot';
param.swingFoot = 'left_foot';
plotWalk(opt_joint_angles_2,robot,param);
% Plot 3rd step
param.support_foot = 'left_foot';
param.swingFoot = 'right_foot';
plotWalk(opt_joint_angles_3,robot,param);
%% Plot Data
% Plot first step
param.support_foot = 'left_foot';
param.swingFoot = 'right_foot';
rCPp1 = plotData(opt_joint_angles_1,param,foot_traj_1);
% Plot 2nd step
param.support_foot = 'right_foot';
param.swingFoot = 'left_foot';
rCPp2 =plotData(opt_joint_angles_2,param,foot_traj_2);
% Plot 3rd step
param.support_foot = 'left_foot';
param.swingFoot = 'right_foot';
rCPp3 = plotData(opt_joint_angles_3,param,foot_traj_3);

rCPp = [rCPp1 rCPp2 rCPp3];

plotZMP(param,rCPp);

%% Pack servo positions
servo_positions = [opt_joint_angles_1,opt_joint_angles_2,opt_joint_angles_3];
save('servo_positions.mat');
out=servo_positions(:);