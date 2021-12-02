function [opt_joint_angles] = inverseKinematics(p,foot_trajectory)
%% Helper Functions
FK = Kinematics();
%% Run optimization
sim_time = length(foot_trajectory);
opt_joint_angles = zeros(p.num_bodies,sim_time);
joints0 = p.initial_conditions;
for i = 1:sim_time
        cost = @(q) costFunctionStatic(q,foot_trajectory,joints0,p,i);
        nlconstraint = @(q) nonlconFoot(q,p,joints0);
        options = optimset('algorithm','sqp');
        [joints_opt] = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint, options);
        joints0 = joints_opt; %warm-start next optimization
        opt_joint_angles(:,i) = joints_opt;
end
end

