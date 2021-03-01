function output = plotRobot(jointAngles)
close all;
%% Build Robot
robot = rigidBodyTree('DataFormat','column');
L1 = 1;
L2 = 1;
L3 = 1;
L4 = 1;
% Link 1
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
% Link 2
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');
% Link 3
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');
% Link 4
body = rigidBody('link4');
joint = rigidBodyJoint('joint4','revolute');
setFixedTransform(joint, trvec2tform([L3,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link3');
% Add end effector
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L4, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link4');

transform = getTransform(robot,[0;0;0;0],'tool','base')
% showdetails(robot)
% %Define Trajecrtory
t = (0:0.2:10)'; % Time
% count = length(t);
center = [0.1 0 0];
radius = 0.1;
theta = t*(2*pi/t(end))/2;
points = center + radius*[-cos(theta) sin(theta) zeros(size(theta))];
%% Plot Robot Configurations
s = size(jointAngles');
count = s(2);
figure
show(robot,jointAngles(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    center = centerOfMass(robot,jointAngles(i,:)');
    plot(center(1),center(2),'.');
    show(robot,jointAngles(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
    plot(center(1),center(2));
% transform = getTransform(robot,[0;0;0;0],'base','tool')

end

