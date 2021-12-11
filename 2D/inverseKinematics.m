function [opt_joint_angles] = inverseKinematics(robot,trajectory,param)
%% Helper Functions
position = @(transform) transform(1:3,4);
rotation = @(transform) transform(1:3,1:3);
%% Run optimization
sim_time = size(trajectory,2);
opt_joint_angles = zeros(param.numBodies,sim_time);
joints0 = param.initialConditions;
for i = 1:sim_time
    cost = @(joint_angles) norm(trajectory(:,i) - position(getTransform(param.robot,joint_angles,param.swingFoot,param.supportFoot)));
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    nlconstraint = @(joint_angles) nonlconFoot(joint_angles,robot,param,joints0);
    [joints_opt] = fmincon(cost,joints0,A,b,Aeq,beq,lb,ub,nlconstraint);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

