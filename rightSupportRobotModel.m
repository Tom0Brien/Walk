function output = rightSupportRobotModel
rightSupportRobot = rigidBodyTree('DataFormat','column');

%right foot
body = rigidBody('right_foot');
joint = rigidBodyJoint('right_ankle_roll ', 'revolute');
body.Mass = 0.205000000000000;
body.CenterOfMass = [0.030800000000000,0.008770000000000,0.011290000000000];
body.Inertia = [5.111249005670000e-04,0.001304219943000,0.001600356100210,-1.853831148134000e-05,-6.611944255527000e-05,-6.774667853889999e-05];
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[0;0;0;1]]);
setFixedTransform(joint,transform);
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'base');

% right Ankle
body = rigidBody('right_ankle');
joint = rigidBodyJoint('right_ankle_pitch  ', 'revolute');
body.Mass = 0.306;
body.CenterOfMass = [-0.015730000000000,5.100000000000000e-04,-0.021970000000000];
body.Inertia = [2.657096439200000e-04,4.401677444590000e-04,3.377297690800000e-04,2.381447176430000e-06,-1.019875646382700e-04,-1.295953861640000e-06];
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[-0.04; 0;  0;1]]);
setFixedTransform(joint,transform);
joint.JointAxis = [0 -1 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'right_foot');

% right Knee
body = rigidBody('right_lower_leg');
joint = rigidBodyJoint('right_knee_pitch  ', 'revolute');
body.Mass = 0.177;
body.CenterOfMass = [-0.1,0.00128,-0.00707];
body.Inertia = [2.721226389810000e-04,0.003365939276850,0.003188010947420,2.292699277416000e-06,1.382949989830000e-04,3.103922526059000e-05];
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[0;0;-0.2;1]]);
setFixedTransform(joint,transform);
joint.JointAxis = [0 -1 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'right_ankle');

% right Hip Pitch
body = rigidBody('right_upper_leg');
body.Mass = 0.387;
body.CenterOfMass = [-0.086820000000000,-9.900000000000000e-04,-0.003220000000000];
body.Inertia = [7.010923891320000e-04,0.005313895148660,0.004974395523190,-1.350555236437000e-06,-5.788145246620000e-04,-2.372857853579000e-05];
joint = rigidBodyJoint('right_hip_pitch', 'revolute');
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[0;0;-0.2;1]]);
setFixedTransform(joint,transform);
joint.JointAxis = [0 -1 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'right_lower_leg');

% right Hip Roll
body = rigidBody('right_hip_roll');
body.Mass = 0.306;
body.CenterOfMass = [0.015730000000000,3.000000000000000e-05,-0.020710000000000];
body.Inertia = [2.491746279200000e-04,4.237120436589999e-04,3.376504538800000e-04,-8.570732235700001e-07,9.592270583826999e-05,3.606376261640000e-06];
joint = rigidBodyJoint('right_hip_roll', 'revolute');
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[0.01;0;0.01;1]]);
setFixedTransform(joint,transform);
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'right_upper_leg');

% right Hip Yaw
body = rigidBody('right_hip_yaw');
body.Mass = 0.067;
body.CenterOfMass = [-0.002180000000000,0.046050000000000,-0.024750000000000];
body.Inertia = [0.001065348115809,0.001785987669820,0.002139389084890,1.016942065902000e-04,-1.146941266165000e-05,5.020539860890000e-06];
joint = rigidBodyJoint('right_hip_yaw', 'revolute');
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[0;0;-0.06;1]]);
setFixedTransform(joint,transform);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(rightSupportRobot, body, 'right_hip_roll');

%Torso
body = rigidBody('torso');
joint = rigidBodyJoint('torso', 'fixed');
transform = inv([[eulerRotation([0 0 0]);[0,0,0]],[0;-0.055;0;1]]);
setFixedTransform(joint,transform);
body.Joint = joint;
addBody(rightSupportRobot, body, 'right_hip_yaw');

%left_hip_yaw
body = rigidBody('left_hip_yaw');
joint = rigidBodyJoint('left_hip_yaw', 'revolute');
transform = [[eulerRotation([0,0,0]);[0,0,0]],[0;0.055;0;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(rightSupportRobot, body, 'torso');

%left_hip_roll
body = rigidBody('left_hip_roll');
joint = rigidBodyJoint('left_hip_roll', 'revolute');
transform = [[eulerRotation([0,0,0]);[0,0,0]],[0;0;-0.06;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'left_hip_yaw');        
%left_upper_leg
body = rigidBody('left_upper_leg');
joint = rigidBodyJoint('left_hip_pitch', 'revolute');
body.Mass = 0.387000000000000;
body.CenterOfMass = [-0.086820000000000,-9.900000000000000e-04,-0.003220000000000];
body.Inertia = [7.010923891320000e-04,0.005313895148660,0.004974395523190,-1.116801963563000e-06,-5.788145246620000e-04,-4.279811466421000e-05];
transform = [[eulerRotation([0,0,0]);[0,0,0]],[0.01;0;0.01;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'left_hip_roll');

%left_knee_pitch
body = rigidBody('left_lower_leg');
joint = rigidBodyJoint('left_knee_pitch', 'revolute');
body.Mass = 0.177000000000000;
body.CenterOfMass = [-0.1,0.00128,-0.00707];
body.Inertia = [2.721226389810000e-04,0.003365939276850,0.003188010947420,9.108591225840000e-07,1.382949989830000e-04,1.427277473941000e-05];
transform = [[eulerRotation([0,0,0]);[0,0,0]],[0;0;-0.2;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'left_upper_leg');

%left_ankle_pitch
body = rigidBody('left_ankle');
joint = rigidBodyJoint('left_ankle_pitch', 'revolute');
body.Mass = 0.306000000000000;
body.CenterOfMass = [-0.015730000000000,5.100000000000000e-04,-0.021970000000000];
body.Inertia = [2.657096439200000e-04,4.401677444590000e-04,3.377297690800000e-04,4.475829223570000e-06,-1.019875646382700e-04,6.205601461640000e-06];
transform = [[eulerRotation([0,0,0]);[0,0,0]],[0;0;-0.2;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [0 1 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'left_lower_leg');
%left_ankle_roll
body = rigidBody('left_foot');
joint = rigidBodyJoint('left_ankle_roll', 'revolute');
body.Mass = 0.205000000000000;
body.CenterOfMass = [0.030800000000000,0.008770000000000,0.011290000000000];
body.Inertia = [5.111249005670000e-04,0.001304219943000,0.001600356100210,-2.205714151866000e-05,-6.611944255527000e-05,-4.300088146110000e-05];
transform = [[eulerRotation([0,0,0]);[0,0,0]],[-0.04;0;0;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [1 0 0];
body.Joint = joint;
addBody(rightSupportRobot, body, 'left_ankle');

%head
body = rigidBody('neck');
joint = rigidBodyJoint('head_yaw', 'revolute');
body.Mass = 2.539;
body.CenterOfMass = [0.005,0.00888,0.10215];
body.Inertia = [5.111249005670000e-04,0.001304219943000,0.001600356100210,-2.205714151866000e-05,-6.611944255527000e-05,-4.300088146110000e-05];
transform = [[eulerRotation([0,0,0]);[0,0,0]],[0;    0;  0.26;1]];
setFixedTransform(joint,transform);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(rightSupportRobot, body, 'torso');
show(rightSupportRobot)
getTransform(rightSupportRobot,homeConfiguration(rightSupportRobot),'left_foot')
output = rightSupportRobot;
end