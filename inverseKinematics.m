function [opt_joint_angles] = inverseKinematics(robot,trajectory,param)
%% Helper Functions
xe = @(kinematics) kinematics.xe;
rotation = @(kinematics) kinematics.R;
RL = @(kinematics) kinematics.RL;
RR = @(kinematics) kinematics.RR;
%% Run optimization
sim_time = length(trajectory);
opt_joint_angles = zeros(param.numBodies,sim_time);
joints0 = param.initialConditions;
for i = 1:sim_time
    cost = @(joint_angles) norm(trajectory(:,i) - xe(kinematics3D(joint_angles,param))) ...
                           + 0.5*trace(eye(3)-rotation(kinematics3D(joint_angles,param))) ...
                           + 0.5*trace(eye(3)-RL(kinematics3D(joint_angles,param))) ...
                           + 0.5*trace(eye(3)-RR(kinematics3D(joint_angles,param)));
    nlconstraint = @(joint_angles) nonlconFoot(joint_angles,robot,param,joints0);
    [joints_opt] = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint);
    joints0 = joints_opt; %warm-start next optimization
    opt_joint_angles(:,i) = joints_opt;
end
end

