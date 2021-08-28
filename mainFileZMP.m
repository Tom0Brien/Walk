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
%% Trajectory Plan Half Step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
% param.initialConditions = opt_joint_angles_1(:,end);
isHalfStep = true;
param.stepCount = 1;
foot_traj_1 = generateFootTrajectory(param,isHalfStep);
[com_x, com_y, zmp_x, zmp_y] = generateCoMTraj(param);
param.com_x = com_x;
param.com_y = com_y;
param.zmp_x = zmp_x;
param.zmp_y = zmp_y;
rCdWw = [com_x(1:param.numSamples);com_y(1:param.numSamples);zeros(1,param.numSamples)];
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_1] = inverseKinematicsZMP(param,foot_traj_1,rCdWw);
%% Trajectory Plan Full Step
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
isHalfStep = false;
param.stepCount = 2;
param.initialConditions = opt_joint_angles_1(:,end);
foot_traj_2 = generateFootTrajectory(param,isHalfStep);
rCdWw = [com_x(param.numSamples+1:param.numSamples*2);com_y(param.numSamples+1:param.numSamples*2);zeros(1,param.numSamples)];
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_2] = inverseKinematicsZMP(param,foot_traj_2,rCdWw);
%% Trajectory Plan Full Step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
isHalfStep = false;
param.stepCount = 3;
param.initialConditions = opt_joint_angles_2(:,end);
foot_traj_3 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_3] = inverseKinematicsZMP(param,foot_traj_3,rCdWw);
%% Plot Walking
% Plot first step
figure
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotWalk(opt_joint_angles_1,robot,param);
% Plot 2nd step
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
plotWalk(opt_joint_angles_2,robot,param);
% Plot 3rd step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotWalk(opt_joint_angles_3,robot,param);
%% Plot Data
% Plot first step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotData(opt_joint_angles_1,param,foot_traj_1);
% Plot 2nd step
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
plotData(opt_joint_angles_2,param,foot_traj_2);
% Plot 3rd step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotData(opt_joint_angles_3,param,foot_traj_3);
%% Plot CoM
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotCoM(opt_joint_angles_1,param);
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
plotCoM(opt_joint_angles_2,param);
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotCoM(opt_joint_angles_3,param);
%% Pack servo positions
servo_positions = [opt_joint_angles_1,opt_joint_angles_2,opt_joint_angles_3];
out=servo_positions(:)