function [com_x, com_y, zmp_x, zmp_y] = generateCoMTraj(param)

% Load params
zc = param.zc; % LIPM height
Ts = param.Ts; % delta time (s)
step_time = param.stepTime; % timing for one step (s)
t_preview = 2; % timing for preview (s)

% Need to be tuned manually
% Tune these two parameters until you get proper CoM trajectory
Qe = 1;
R = 1e-2;

footstep = param.footstep;

t_calc = length(footstep) * step_time - t_preview - Ts;

% Generating ZMP trajectory
[zmp_x, zmp_y] = createZmpTrajectory(footstep, Ts, step_time);

% Getting parameter and gain
[A_d, B_d, C_d, Gi, Gx, Gd] = previewControlParams(zc, Ts, t_preview, Qe, R);

% Simulating for t_calc seconds
[com_x, com_y] = previewControl(zmp_x, zmp_y, Ts, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd);
end
