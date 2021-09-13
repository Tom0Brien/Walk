clc;
clear;
close all;
addpath('./transforms');
addpath('./ZMP');
%% Forward kinematics
robot = importrobot('NUgus.urdf');
robot.DataFormat = 'column';
%% Params
p = gaitParameters(robot);
%% Plot footsteps
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
plotFootsteps(p);
%% ZMP Trajectory
[com_x, com_y, zmp_x, zmp_y] = generateCoMTraj(p);
p.com_x = com_x;
p.com_y = com_y;
p.zmp_x = zmp_x;
p.zmp_y = zmp_y;
%% For loop
is_half_step = true;
opt_joint_angles = [];
foot_traj = [];
opt_joint_angles_temp = p.initial_conditions;
for i=1:(length(p.footsteps)- 2)
    p.step_count = i;   
    %get swing foot trajectory
    p.initial_conditions = opt_joint_angles_temp(:,end);
    foot_traj_temp = generateFootTrajectory(p);
    foot_traj = [foot_traj foot_traj_temp];
    %get com traj for step
    rCdWw = [com_x((i-1)*p.N+1:p.N*i);com_y((i-1)*p.N+1:p.N*i);zeros(1,p.N)];
    %perform inverse kinematics
    opt_joint_angles_temp = inverseKinematicsZMP(p,foot_traj_temp,rCdWw);
    opt_joint_angles = [opt_joint_angles opt_joint_angles_temp];
    %switch feet
    if(p.support_foot == "left_foot")
        p.support_foot = 'right_foot';
        p.swingFoot = 'left_foot';
    else
        p.support_foot = 'left_foot';
        p.swingFoot = 'right_foot';
    end
end
%% Plot Walking
% Plot first step
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
for i=1:(length(p.footsteps)- 2)
    plotWalk(opt_joint_angles(:,(i-1)*p.N+1:p.N*i),robot,p);
    if(p.support_foot == "left_foot")
        p.support_foot = 'right_foot';
        p.swingFoot = 'left_foot';
    else
        p.support_foot = 'left_foot';
        p.swingFoot = 'right_foot';
    end
end
%% Plot Data
% % Plot first step
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
rCPp = [];
for i=1:(length(p.footsteps)- 2)
    rCPp = [rCPp plotData(opt_joint_angles(:,(i-1)*p.N+1:p.N*i),p,foot_traj(:,(i-1)*p.N+1:p.N*i))];
    if(p.support_foot == "left_foot")
        p.support_foot = 'right_foot';
        p.swingFoot = 'left_foot';
    else
        p.support_foot = 'left_foot';
        p.swingFoot = 'right_foot';
    end
end
%% Plot ZMP
 plotZMP(p,rCPp);

%% Pack servo positions
save('opt_joint_angles.mat');
out=opt_joint_angles(:);