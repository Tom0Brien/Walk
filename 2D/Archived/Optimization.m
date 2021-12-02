clear;
clc;
close all; 
%% Forward Kinematics (4 Link Robot)
% Link Lengths
L1 = 0.2;
L2 = 0.2;
L3 = 0.2;
L4 = 0.2;
L5 = 0.2; % Distance from right knee to CoM of toso
param.L1 = L1;
param.L2 = L2;
param.L3 = L3;
param.L4 = L4;

% Define mass for each link
M1 = 0.25;
M2 = 0.25;
M3 = 0.25;
M4 = 0.25;
M5 = 0.5;
% Define distance to CoM from joint for each link
r1 = L1/2;
r2 = L2/2;
r3 = L3/2;
r4 = L4/2;
% Complete Homogeneous Transformation
T = @(j) [cos(j(1)+j(2)+j(3)+j(4)) -sin(j(1)+j(2)+j(3)+j(4)) 0 L1*cos(j(1))+L2*cos(j(1)+j(2))+L3*cos(j(1)+j(2)+j(3))+L4*cos(j(1)+j(2)+j(3)+j(4));
          sin(j(1)+j(2)+j(3)+j(4)) cos(j(1)+j(2)+j(3)+j(4)) 0 L1*sin(j(1))+L2*sin(j(1)+j(2))+L3*sin(j(1)+j(2)+j(3))+L4*sin(j(1)+j(2)+j(3)+j(4));
          0 0 1 0;
          0 0 0 1;]
% End effector position [x,y]
end_effector = @(j) [L1*cos(j(1))+L2*cos(j(1)+j(2))+L3*cos(j(1)+j(2)+j(3))+L4*cos(j(1)+j(2)+j(3)+j(4)), L1*sin(j(1))+L2*sin(j(1)+j(2))+L3*sin(j(1)+j(2)+j(3))+L4*sin(j(1)+j(2)+j(3)+j(4))];
%% Track a half-circle path
% Define Trajectory
t = (0:0.1:7.5)'; % Time
count = length(t);
% center = [0.01 0];
% radius = 0.02;
% theta = t*(2*pi/t(end))/2;
for t =0:25
    points(t+1,:) = [0 t*0.02/25];
end
for t = 0:25
    points(25+t,:) = [t*0.02/25 0.02];
end
for t =0:25
    points(50+t,:) = [0.02 0.02-t*0.02/25];
end
param.points = points;
%% Quasi-static CoM Constraint within the cost function
% Centre of mass function: CoM = x*m/m
CoM = @(j) (r1*cos(j(1))*M1 + ... Link 1
           (L1*cos(j(1))+r2*cos(j(1)+j(2)))*M2 + ... Link 2
           (L1*cos(j(1))+ L5*cos(j(1)+j(2)))*M5  + ... Torso
           (L1*cos(j(1))+ L2*cos(j(1)+j(2))+r3*cos(j(1)+j(2)+j(3)))*M3 + ... Link 3
           (L3*cos(j(1))+ L2*cos(j(1)+j(2))+L3*cos(j(1)+j(2)+j(3))+ ... Link 4
           r4*cos(j(1)+j(2)+j(3)+j(4)))*M4)/(M1+M2+M3+M4+M5);
       
%% Find optimal solution at each point along the half-circle
joint_angles = zeros(count,4);
params0 = [(80/180)*pi,(40/180)*pi,(187/180)*pi,(320/180)*pi];
% Step 1
for i = 1:count - 1 
    ed = points(i,:); %Desired end effector position
    cost = @(joint_angles) norm(ed - end_effector(joint_angles)) + norm(CoM(joint_angles));
    A = [];
    B = [];
    Aeq = [];
    Beq = [];
    lb = [0,0,0,(270/180)*pi];
    ub = [pi/2,2*pi,2*pi,2*pi];
    b = [pi/2;0;2*pi;(270/180)*pi];
    [params_opt, costVal] = fmincon(cost,params0,A,B,Aeq,Beq,lb,ub);
    params0 = params_opt; %Set the next initial conditons to the optimal solution found
    opt_joint_angles(i,:) = params_opt;
end
%% Plot the robot walking
plotRobot(opt_joint_angles,param);