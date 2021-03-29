clc;
clear;
close all;
robot = importrobot('NUgus.urdf');
robot.DataFormat = 'column';
%% 1: Trajectory Plan
trajectory = zeros(3,10);
for t = 1:10
    trajectory(:,t) = [0;0;t - 1]*0.01;
end
for t = 1:10
    trajectory(:,t+10) = trajectory(:,10)+[t - 1;0;0]*0.01;
end
for t = 1:10
    trajectory(:,t+20) = trajectory(:,20)+[0;0;t - 1]*-0.01;
end

%shift to right foot
trajectory = trajectory + [0.0294;-0.08;-0.4443];
sim_time = length(trajectory);
%% 2: Inverse Kinematics
opt_joint_angles = zeros(20,sim_time);
joints0 = initialConditions;
for i = 1:sim_time
    rotation = [-0.0292   -0.9824    0.1843;
                -0.9976    0.0171   -0.0669;
                0.0626   -0.1858   -0.9806;
                0 0 0];

    ed = [rotation, [trajectory(:,i);1]]; %Desired end effector position
    cost = @(joint_angles) norm(ed - Kinematics3D(joint_angles));
%     + norm(centerOfMass(robot,joint_angles).*[1,0,0]);
    [joints_opt, costVal] = fmincon(cost,joints0);
    joints0 = joints_opt; %Set the next initial conditons to the optimal solution found
    opt_joint_angles(:,i) = joints_opt;
end

%% Plot
param.trajectory = trajectory;
plot3DRobot(opt_joint_angles,param);