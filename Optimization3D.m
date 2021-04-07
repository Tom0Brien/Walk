 clc;
clear;
close all;
%% Load robot from urdf
torsoRobot = importrobot('NUgus.urdf');
torsoRobot.DataFormat = 'column';
getTransform(torsoRobot,zeros(20,1),'left_foot','right_foot');
show(torsoRobot)
figure(1);
N = 13;
leftFootRobot = leftSupportRobotModel;
leftFootRobot.DataFormat = 'column';
getTransform(leftFootRobot,zeros(N,1),'left_foot','right_foot');
Kinematics3D(zeros(20,1)).leftToRightFoot

%% 1: Trajectory Plan
stepLength = 0.1;
stepHeight = 0.1;
numPoints = 30;
%shift to right foot
trajectory = evalFootGait(stepLength,stepHeight,numPoints) - [0;0.15;0];
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:))
sim_time = length(trajectory);
%% 2: Inverse Kinematics
opt_joint_angles = zeros(N,sim_time);
%Step 1
joints0 = initialConditions;
joints0 = zeros(N,1)
torso_transforms = zeros(4,4,30);
for i = 1:sim_time
    position = @(transform) transform(1:3,4);
    rotation = @(transform) transform(1:3,1:3);
    cost = @(joint_angles) norm(trajectory(:,i) - position(getTransform(leftFootRobot,joint_angles,'right_foot'))) ...
                            + 0.5*trace(eye(3)-eulerRotation([0,0,-pi/2])*rotation(getTransform(leftFootRobot,joint_angles,'right_foot')));
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    nlconstraint = @(joint_angles) nonlconFoot(joint_angles,leftFootRobot);
    [joints_opt, costVal] = fmincon(cost,joints0,A,b,Aeq,beq,lb,ub,nlconstraint);
    joints0 = joints_opt; %Set the next initial conditons to the optimal solution found
    opt_joint_angles(:,i) = joints_opt;
end

%% Plot
param.trajectory = trajectory;
param.N = N;
plot3DRobot(opt_joint_angles,leftFootRobot,param);