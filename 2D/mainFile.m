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
trajectory1 = generateFootTrajectory(param,isHalfStep);  
%% Inverse Kinematics
[opt_joint_angles] = inverseKinematics(robot,trajectory1,param);
%% Generate Trajectory 2
isHalfStep = false;
param.swingFoot = 'left_lower_leg';
param.supportFoot = 'right_lower_leg';
param.initialConditions = opt_joint_angles(:,end);
trajectory2 = generateFootTrajectory(param,isHalfStep); 
%% Inverse Kinematics 2
[opt_joint_angles_2] = inverseKinematics(robot,trajectory2,param);
%% Generate Trajectory 3
isHalfStep = false;
param.swingFoot = 'right_lower_leg';
param.supportFoot = 'left_lower_leg';
param.initialConditions = opt_joint_angles_2(:,end);
trajectory3 = generateFootTrajectory(param,isHalfStep); 
%% Inverse Kinematics 3
[opt_joint_angles_3] = inverseKinematics(robot,trajectory3,param);
%% Plot the robot walking
plot3DRobot([opt_joint_angles opt_joint_angles_2 opt_joint_angles_3 opt_joint_angles_2 opt_joint_angles_3 opt_joint_angles_2 opt_joint_angles_3],robot,param,trajectory1);
