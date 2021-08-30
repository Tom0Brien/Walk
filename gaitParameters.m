function param = gaitParameters(robot)
param.num_bodies = robot.NumBodies;
param.step_height = 0.05;
param.step_length = 0.36; %0.15
param.step_width = 0.18;
param.step_time = 0.46;
param.Ts = 0.01;
param.N_samples = param.step_time/param.Ts;
param.initial_conditions = initialConditions;
param.robot = robot;
%zmp params
param.g = 9.81;        % Gravity
param.zc = 0.46;     % Center of Mass Height (constant)
param.footstep = [0.0 0.0 0.0; 
            param.step_length/2 -param.step_width/2 0.0; 
            param.step_length*2/2 param.step_width/2 0.0; 
            param.step_length*3/2 -param.step_width/2 0.0;
            param.step_length*4/2 param.step_width/2 0.0;
            param.step_length*5/2 -param.step_width/2 0.0;
            param.step_length*6/2 param.step_width/2 0.0;
            ];
param.stepCount = 1;
end