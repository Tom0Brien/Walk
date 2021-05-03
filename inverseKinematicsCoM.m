function [opt_joint_angles] = inverseKinematicsCoM(robot,trajectory,param)
%% Helper Functions
position = @(transform) transform(1:3,4);
rotation = @(transform) transform(1:3,1:3);
%% Run optimization
sim_time = length(trajectory);
opt_joint_angles = zeros(param.numBodies,sim_time);
joints0 = param.initialConditions;
for i = 1:sim_time
    Htsupport = @(joint_angles) getTransform(param.robot,joint_angles,'torso',param.supportFoot);
    Htswing = @(joint_angles) getTransform(param.robot,joint_angles,'torso',param.swingFoot);
    cost = @(joint_angles) norm([trajectory(:,i);1] - [centerOfMass(param.robot,joint_angles);1]) ...
                           + 0.5*trace(eye(3)-rotation(getTransform(robot,joint_angles,param.swingFoot,param.supportFoot))) ...
                           + 0.5*trace(eye(3)-rotation(getTransform(robot,joint_angles,param.swingFoot)));
%     + ...
%                            norm(Htswing(joint_angles)*[trajectory(:,i);1] - Htswing(joint_angles)*[centerOfMass(param.robot,joint_angles);1]) ...
%                            + 0.5*trace(eye(3)-rotation(getTransform(robot,joint_angles,param.swingFoot,param.supportFoot)));
    nlconstraint = @(joint_angles) nonlconCoM(joint_angles,robot,param,joints0);
    [joints_opt] = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

