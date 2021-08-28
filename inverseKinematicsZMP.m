function [opt_joint_angles] = inverseKinematicsZMP(p,foot_traj,com_traj)
%% Run optimization
FK = Kinematics();
opt_joint_angles = zeros(p.numBodies,p.numSamples);
joints0 = p.initialConditions;
for i = 1:p.numSamples
    cost = @(q) costFunction(q,foot_traj,com_traj,p,i);
    nlconstraint = @(q) nonlconCoM(q);
    [joints_opt] = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

