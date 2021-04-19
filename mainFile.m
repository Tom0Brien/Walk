clc;
clear;
close all;
%% Forward kinematics
robot = importrobot('NUgus.urdf');
robot.DataFormat = 'column';
%% Params
param = gaitParameters(robot);
%% Trajectory Plan Half Step
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
isHalfStep = true;
trajectory_1 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_1] = inverseKinematics(robot,trajectory_1,param);
%% Trajectory Plan Full Step
param.initialConditions = opt_joint_angles_1(:,end);
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
isHalfStep = false;
trajectory_2 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Full Step - support foot -> right
[opt_joint_angles_2] = inverseKinematics(robot,trajectory_2,param);
%% Trajectory Plan Full Step
param.initialConditions = opt_joint_angles_2(:,end);
param.supportFoot = 'left_foot';
param.swingFoot = 'right_foot';
isHalfStep = false;
trajectory_3 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Full Step - support foot -> right
[opt_joint_angles_3] = inverseKinematics(robot,trajectory_3,param);
%% Plot
opt_joint_angles = [opt_joint_angles_1,opt_joint_angles_2,opt_joint_angles_3,opt_joint_angles_2,opt_joint_angles_3];
param.numSamples = length(opt_joint_angles);
param.trajectory = [trajectory_1,trajectory_2,trajectory_3,trajectory_2,trajectory_3];
plot3DRobot(opt_joint_angles,robot,param);