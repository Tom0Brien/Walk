function [opt_joint_angles] = inverseKinematicsCoM(robot,trajectory,param)
%% Helper Functions
position = @(transform) transform(1:3,4);
rotation = @(transform) transform(1:3,1:3);
%% Run optimization
sim_time = length(trajectory);
opt_joint_angles = zeros(param.NumBodies,sim_time);
joints0 = param.initialConditions;
for i = 1:sim_time
    cost = @(joint_angles) norm(trajectory(:,i) - [1,1,0,0]*getTransform(param.robot,joint_angles,'torso',param.supportFoot)*[centerOfMass(param.robot,joint_angles);1]) + ...
                           + 0.5*trace(eye(3)-rotation(getTransform(robot,joint_angles,param.swingFoot,param.supportFoot)));
    [joints_opt] = fmincon(cost,joints0);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

