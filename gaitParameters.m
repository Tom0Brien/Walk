function param = gaitParameters(robot)
param.numBodies = robot.NumBodies;
param.stepHeight = 0.08;
param.stepLength = 0.1;
param.stepWidth = 0.13;
param.stepTime = 1;
param.Ts = 0.1;
param.numSamples = param.stepTime/param.Ts;
param.initialConditions = initialConditions;
param.robot = robot;
%zmp params
param.g = 9.81;        % Gravity
param.zc = 0.46;     % Center of Mass Height (constant)
param.footstep = [0.0 0.0 0.0; 
            param.stepLength/2 -param.stepWidth/2 0.0; 
            param.stepLength*2/2 param.stepWidth/2 0.0; 
            param.stepLength*3/2 -param.stepWidth/2 0.0;
            param.stepLength*4/2 param.stepWidth/2 0.0;
            ];
param.stepCount = 1;
end