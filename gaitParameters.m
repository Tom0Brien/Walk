function param = gaitParameters(robot)
param.numBodies = robot.NumBodies;
param.stepHeight = 0.05;
param.stepLength = 0.2;
param.stepWidth = 0.15;
param.stepTime = 2;
param.Ts = 0.1;
param.numSamples = param.stepTime/param.Ts;
param.initialConditions = zeros(20,1);
param.robot = robot;
end