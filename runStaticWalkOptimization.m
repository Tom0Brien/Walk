clc;
clear;
close all;
addpath('./transforms');
%% Forward kinematics
robot = importrobot('NUgus.urdf');
robot.DataFormat = 'column';
%% Params
p = gaitParameters(robot);
%% Broad search optimization
all_results = [];
iteration = 0;
for step_length_x=0.05:0.05:0.5
%     for step_time = 0.05:0.05:0.5
    %update gait parameters
    p.step_length_x = step_length_x;
%     p.step_time = step_time;
    p.footsteps = generateFootsteps(p);
    p.iteration = iteration;
    opt_joint_angles = [];
    iteration = iteration + 1;
    %initial conditions
    p.support_foot = 'left_foot';
    p.swingFoot = 'right_foot';
    opt_joint_angles = [];
    foot_traj = [];
    p.initial_conditions = initialConditions;
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
    %% Run simulation
    save('controllers/walk_controller/data.mat','p','opt_joint_angles');
    path_to_webots = "X:\Webots\Webots\msys64\mingw64\bin\webots.exe";
    path_to_world = "X:\Walk\worlds\kid.wbt";
    open_webots = path_to_webots + " " + path_to_world;
    system(open_webots)
    result = load('controllers/walk_controller/result.mat');
    %update results param
    result.distance_travelled = result.position(end,1);
    result.step_length_x = step_length_x;
%     result.step_time = step_time;
    result.iteration = p.iteration;
    %store results
    all_results = [all_results;result];
%     end
end

%% Plot the results
close all;
plot_results = zeros(2,size(all_results,1));
for i=1:size(all_results,1)
    plot_results(1,i) = all_results(i).step_length_x;
%     plot_results(2,i) = all_results(i).step_time;
    plot_results(2,i) = all_results(i).distance_travelled;
end
plot(plot_results(1,:),plot_results(2,:),'LineWidth',3)
title('Static Walk Results');
xlabel('Step Length X [m]');
% ylabel('Step Time [s]');
ylabel('Distance Travelled [m]');
grid on;

    
