function [com_x, com_y, zmp_x, zmp_y] = generateCoMTraj(param)

% Load params
zc = param.zc; 
Ts = param.Ts; 
step_time = param.step_time; 
t_preview = param.step_time*2; 
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
