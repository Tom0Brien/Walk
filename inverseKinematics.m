function [opt_joint_angles] = inverseKinematics(p,foot_trajectory)
%% Helper Functions
FK = Kinematics();
%% Run optimization
sim_time = length(foot_trajectory);
opt_joint_angles = zeros(p.num_bodies,sim_time);
joints0 = p.initial_conditions;
for i = 1:sim_time
    cost = @(q) norm(foot_trajectory(:,i) - FK.xe(q,p)) ...
                           + 0.5*trace(eye(3)-FK.R(q,p)) ...
                           + 0.5*trace(eye(3)-FK.RL(q,p)) ...
                           + 0.5*trace(eye(3)-FK.RR(q,p)) + ...
                           0.0001*norm(joints0-q);
    nlconstraint = @(q) nonlconFoot(q,p,joints0);
    [joints_opt] = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

