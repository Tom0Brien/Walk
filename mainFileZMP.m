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
p = gaitParameters(robot);
%% ZMP Trajectory
[com_x, com_y, zmp_x, zmp_y] = generateCoMTraj(p);
p.com_x = com_x;
p.com_y = com_y;
p.zmp_x = zmp_x;
p.zmp_y = zmp_y;
%% For loop
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
is_half_step = true;
opt_joint_angles = [];
foot_traj = [];
for i=1:(length(p.footstep)- 2)
    p.stepCount = i;   
    %get swing foot trajectory
    foot_traj_temp = generateFootTrajectory(p,is_half_step);
    foot_traj = [foot_traj foot_traj_temp];
    %get com traj for step
    rCdWw = [com_x((i-1)*p.N+1:p.N*i);com_y((i-1)*p.N+1:p.N*i);zeros(1,p.N)];
    %perform inverse kinematics
    opt_joint_angles_temp = inverseKinematicsZMP(p,foot_traj_temp,rCdWw);
    opt_joint_angles = [opt_joint_angles opt_joint_angles_temp];
    %switch feet
    disp(p.support_foot == "left_foot")
    if(p.support_foot == "left_foot")
        disp('now right')
        p.support_foot = 'right_foot';
        p.swingFoot = 'left_foot';
    else
        disp('now left')
        p.support_foot = 'left_foot';
        p.swingFoot = 'right_foot';
    end
    p.initial_conditons = opt_joint_angles_temp(:,end);
    is_half_step = false;
end

%% Plot Walking
% Plot first step
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
for i=1:(length(p.footstep)- 2)
    plotWalk(opt_joint_angles(:,(i-1)*p.N+1:p.N*i),robot,p);
    if(p.support_foot == "left_foot")
        disp('now right')
        p.support_foot = 'right_foot';
        p.swingFoot = 'left_foot';
    else
        disp('now left')
        p.support_foot = 'left_foot';
        p.swingFoot = 'right_foot';
    end
end
%% Plot Data
% % Plot first step
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
rCPp = [];
for i=1:(length(p.footstep)- 2)
    rCPp = [rCPp plotData(opt_joint_angles(:,(i-1)*p.N+1:p.N*i),p,foot_traj(:,(i-1)*p.N+1:p.N*i))];
    if(p.support_foot == "left_foot")
        disp('now right')
        p.support_foot = 'right_foot';
        p.swingFoot = 'left_foot';
    else
        disp('now left')
        p.support_foot = 'left_foot';
        p.swingFoot = 'right_foot';
    end
end
%% Plot ZMP
 plotZMP(p,rCPp);

%% Pack servo positions
servo_positions = [opt_joint_angles_1,opt_joint_angles_2,opt_joint_angles_3];
save('servo_positions.mat');
out=servo_positions(:);