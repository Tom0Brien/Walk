clc;
clear;
close all;
addpath('./transforms');
%% Forward kinematics
robot = importrobot('NUgus.urdf');
% smimport('NUgus.urdf')
robot.DataFormat = 'column';
%% Params
p = gaitParameters(robot);
%% Plot footsteps
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
plotFootsteps(p);
%% For loop
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
save('controllers/walk_controller/data.mat','p','opt_joint_angles');
servos = opt_joint_angles(:,1:3*p.N);
out=opt_joint_angles(:);

%% Run simulation
if(p.run_simulation)
    path_to_webots = "X:\Webots\Webots\msys64\mingw64\bin\webots.exe";
    path_to_world = "X:\Walk\worlds\kid.wbt";
    open_webots = path_to_webots + " " + path_to_world;
    system(open_webots)
    % Compare Simulation Results
    simulationResultsStatic;
end