clear;
clc;
close all;

robot = importrobot('NUgus.urdf');
robot.DataFormat = 'column';
show(robot)
q0 = homeConfiguration(robot);
getTransform(robot,q0,'right_ankle','torso')

%% Create a trajectory (Circle)

t = (0:0.1:10)'; % Time
count = length(t);
center = [0.05 -0.055000063267949 -0.5];
radius = 0.05;
theta = -t*(2*pi/t(end))/2;
points = center + radius*[-cos(theta) zeros(size(theta)) -sin(theta)];

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = inverseKinematics('RigidBodyTree', robot);

endEffector = 'Body16';

qInitial = [0,0,0,0.2945004,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]';

 % Use home configuration as the initial guess
weights = [0, 0, 0, 1, 1, 1];
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    %Store CoM for this configuration
    com(i,:) = centerOfMass(robot,qs(i,:)');
    % Start from prior solution
    qInitial = qSol;
end

figure
% show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot3(points(:,1),points(:,2),points(:,3),'-r')
axis([-1 1 -1 1])

%% Animate
framesPerSecond = 60;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    fig = plot(com(i,1),com(i,2),'-o');
    fig.MarkerEdgeColor = 'b';
    drawnow
    waitfor(r);
end

% plot(com(:,1),com(:,2),'o')