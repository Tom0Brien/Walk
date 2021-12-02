function output = generateRobot

L1 = 0.2;
L2 = 0.2;
L3 = 0.2;
L4 = 0.2;
%% Build Robot
robot = rigidBodyTree('DataFormat','column');
% Link 1
body = rigidBody('right_hip');
body.Mass = 0.1;
body.CenterOfMass = [0 0 0];
joint = rigidBodyJoint('right_hip_pitch', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'base');
% Link 2
body = rigidBody('right_upper_leg');
body.Mass = 0.1;
body.CenterOfMass = [0 0 -L1/2];
joint = rigidBodyJoint('right_knee','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L1]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'right_hip');
% Link 3
body = rigidBody('right_lower_leg');
body.Mass = 0.1;
body.CenterOfMass = [0 0 -L2/2];
joint = rigidBodyJoint('right_ankle','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L2]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'right_upper_leg');
% Link 4
body = rigidBody('left_hip');
body.Mass = 0.1;
body.CenterOfMass = [0 0 0];
joint = rigidBodyJoint('left_hip_pitch','revolute');
setFixedTransform(joint, trvec2tform([0,0,0]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'base');
%Link 5
body = rigidBody('left_upper_leg');
body.Mass = 0.1;
body.CenterOfMass = [0 0 -L1/2];
joint = rigidBodyJoint('left_knee','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L1]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'left_hip');
%Link 6
body = rigidBody('left_lower_leg');
body.Mass = 0.1;
body.CenterOfMass = [0 0 -L2/2];
joint = rigidBodyJoint('left_ankle','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L2]));
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(robot, body, 'left_upper_leg');
%Torso
body = rigidBody('head');
body.Mass = 0.2;
body.CenterOfMass = [0 0 0.15];
joint = rigidBodyJoint('head_joint','fixed');
setFixedTransform(joint, trvec2tform([0,0,0.3]));
body.Joint = joint;
addBody(robot, body, 'base');

output = robot

end

