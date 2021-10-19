clc;
clear;
close all;
addpath('./transforms');
%% Forward kinematics
robot = importrobot('robot.urdf');
% smimport('NUgus.urdf')
robot.DataFormat = 'column';
%% Params
p = gaitParameters(robot);
%% For loop
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
opt_joint_angles = [];
foot_traj = [];
opt_joint_angles_temp = p.initial_conditions;
for i=1:(length(p.footsteps)- 2)
    p.step_count = i;   
    %get swing foot trajectory
    p.initial_conditions = opt_joint_angles_temp(:,end);
    foot_traj_temp = generateFootTrajectory(p);
    foot_traj = [foot_traj foot_traj_temp];
    %perform inverse kinematics
    opt_joint_angles_temp = inverseKinematics(p,foot_traj_temp);
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
% %% Plot Walking
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

%% Pack servo positions
out=opt_joint_angles(:)