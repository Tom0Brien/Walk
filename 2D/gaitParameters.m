function param = gaitParameters(robot)
param.numBodies = robot.NumBodies - 1;
param.stepHeight = 0.04;
param.stepLength = 0.08;
param.stepWidth = 0;
param.stepTime = 0.5;
param.Ts = 0.05;
param.numSamples = param.stepTime/param.Ts;
param.initialConditions = zeros(param.numBodies,1);
param.robot = robot;
end