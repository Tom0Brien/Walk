function output = plot3DRobot(jointAngles)
close all;
%% Build Robot
robot = rigidBodyTree('DataFormat','column');
L1 = 1;
L2 = 1;
L3 = 0.25;
L4 = 0.25;
L5 = 0.25;
L6 = 0.25;
L7 = 1;
L8 = 1;

% Left Leg
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'base');
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([0,0,L1]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link1');
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([0,0,L2]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'link2');

%Hip
body = rigidBody('link4');
joint = rigidBodyJoint('joint4','fixed');
setFixedTransform(joint, trvec2tform([0,0,L3]));
body.Joint = joint;
addBody(robot, body, 'link3');

body = rigidBody('link5');
joint = rigidBodyJoint('joint5','fixed');
setFixedTransform(joint, trvec2tform([0,L4,0]));
body.Joint = joint;
addBody(robot, body, 'link4');

body = rigidBody('link6');
joint = rigidBodyJoint('joint6','fixed');
setFixedTransform(joint, trvec2tform([0,L5,0]));
body.Joint = joint;
addBody(robot, body, 'link5');

body = rigidBody('link7');
joint = rigidBodyJoint('joint7','fixed');
setFixedTransform(joint, trvec2tform([0,0,-L6]));
body.Joint = joint;
addBody(robot, body, 'link6');

%Right leg
body = rigidBody('link8');
joint = rigidBodyJoint('joint8','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L7]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link7');

body = rigidBody('link9');
joint = rigidBodyJoint('joint9','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L8]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link8');

%Add torso body/arms/head as one link
body = rigidBody('torso');
joint = rigidBodyJoint('hips','fixed');
setFixedTransform(joint, trvec2tform([0,0,0.25]));
body.Joint = joint;
addBody(robot, body, 'link5');

transform = getTransform(robot,[0;0;0;0;0],'link8','base')
% showdetails(robot)
% %Define Trajecrtory
t = (0:0.2:10)'; % Time
% count = length(t);
 center = [0.5 0 0];
 radius = 0.25;
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
    center = centerOfMass(robot,jointAngles(i,:)')
    plot(center(1),center(2),'.');
    show(robot,jointAngles(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
    plot(center(1),center(2));
% transform = getTransform(robot,[0;0;0;0],'base','tool')

end

