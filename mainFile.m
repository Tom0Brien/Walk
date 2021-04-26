lc;
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
param.supportFoot = 'right_foot';
param.swingFoot = 'left_foot';
isHalfStep = false;
param.initialConditions = opt_joint_angles_1(:,end);
trajectory_1 = generateFootTrajectory(param,isHalfStep);
%% Inverse Kinematics - Half Step - support foot -> left
[opt_joint_angles_2] = inverseKinematics(robot,trajectory_1,param);
%% Plot
opt_joint_angles = [opt_joint_angles_1 opt_joint_angles_2]
param.numSamples = size(opt_joint_angles);
plot3DRobot(opt_joint_angles,robot,param);