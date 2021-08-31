function [opt_joint_angles] = inverseKinematicsZMP(p,foot_traj,com_traj)
%% Run optimization
FK = Kinematics();
opt_joint_angles = zeros(p.num_bodies,p.N);
joints0 = p.initial_conditions;
for i = 1:p.N
    cost = @(q) costFunction(q,foot_traj,com_traj,p,i);
    nlconstraint = @(q) nonlconCoM(q);
    [joints_opt] = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

