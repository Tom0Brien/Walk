function param = gaitParameters(robot)
param.NumBodies = robot.NumBodies;
param.stepHeight = 0.12;
param.stepLength = 0.04;
param.stepWidth = 0.15;
param.stepTime = 0.5;
param.Ts = 0.05;
param.numSamples = param.stepTime/param.Ts;
param.initialConditions = initialConditions;
end