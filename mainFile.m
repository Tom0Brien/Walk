clc;
clear;
close all;
%% Forward kinematics
robot = importrobot('NUgus.urdf');
robot.DataFormat = 'column';
%% Params
param = gaitParameters(robot);
%% Trajectory Plan Half Step
param.initialConditions = initialConditions;
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
% param.initialConditions = opt_joint_angles_1(:,end);
isHalfStep = true;
trajectory_1 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_1] = inverseKinematics(robot,trajectory_1,param);
%% Trajectory Plan Full Step
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
isHalfStep = false;
param.initialConditions = opt_joint_angles_1(:,end);
trajectory_2 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_2] = inverseKinematics(robot,trajectory_2,param);
%% Trajectory Plan Full Step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
isHalfStep = false;
param.initialConditions = opt_joint_angles_2(:,end);
trajectory_3 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_3] = inverseKinematics(robot,trajectory_3,param);
%% Plot Walking
% Plot first step
figure
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotWalk(opt_joint_angles_1,robot,param,trajectory_1);
% Plot 2nd step
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
plotWalk(opt_joint_angles_2,robot,param,trajectory_2);
% Plot 3rd step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotWalk(opt_joint_angles_3,robot,param,trajectory_3);
%% Plot Data
% Plot first step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotData(opt_joint_angles_1,robot,param,trajectory_1);
% Plot 2nd step
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
plotData(opt_joint_angles_2,robot,param,trajectory_2);
% Plot 3rd step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotData(opt_joint_angles_3,robot,param,trajectory_3);
%% Plot CoM
plotCoM(opt_joint_angles_1,param);
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
plotCoM(opt_joint_angles_2,param);
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
plotCoM(opt_joint_angles_3,param);
%% Pack servo positions
servo_positions = [opt_joint_angles_1,opt_joint_angles_2,opt_joint_angles_3];
