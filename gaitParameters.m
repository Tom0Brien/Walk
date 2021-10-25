function p = gaitParameters(robot)
% p.walk_command = [0.3;0.0;0];
% p.step_time = 0.75;
% p.step_length_x = p.step_time*p.walk_command(1);
% p.step_length_y = p.step_time*p.walk_command(2);

p.step_time = 0.4;
p.step_length_x = 0.25;
p.step_length_y = 0.0;
p.step_height = 0.045;
p.step_width = 0.16;
p.Ts = 0.01;

% simulation params
p.walk_time = 5;
p.iteration = 0; % used in optimization
p.export = false;
p.run_simulation = true;

p.num_bodies = robot.NumBodies;
p.foot_z_offset = 0.035;
p.N = floor(p.step_time/p.Ts);
p.initial_conditions = initialConditions;
p.robot = robot;
p.num_footsteps = 4;
p.footsteps = generateFootsteps(p);
        
%zmp params
p.g = 9.81;        % Gravity
p.zc = 0.495;     % Center of Mass Height (constant)
p.t_preview = p.step_time*2; 

Qe = 1;
R = 1e-2;

% Matrix A, B, C, and D from cart table model
A = [0 1 0;
     0 0 1;
     0 0 0];
B = [0;
     0;
     1];
C = [1 0 -p.zc/p.g];
D = 0;

% Convert continuous system to discrete system
sys_c = ss(A, B, C, D);            
sys_d = c2d(sys_c, p.Ts);
[p.A_d, p.B_d, p.C_d, p.D_d] = ssdata(sys_d);

% A, B, C matrix for LQI control
A_tilde = [1 p.C_d*p.A_d;
           zeros(3,1) p.A_d];
B_tilde = [p.C_d*p.B_d;
           p.B_d];
C_tilde = [1 0 0 0];

% Q matrix
Q = [Qe 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

% Find optimal gain and ricatti equation with dlqr function
% K = optimal gain
% P = ricatti equation
[K, P] = dlqr(A_tilde, B_tilde, Q, R);

p.Gi = K(1);
p.Gx = K(2:end);

% Calculating preview gain
N = 0:p.Ts:p.t_preview;
p.Gd = zeros(1, length(N));
p.Gp(1,1) = -p.Gi;

Ac_tilde = A_tilde - B_tilde * K;

I_tilde = [1;0;0;0];
X_tilde = -Ac_tilde'*P*I_tilde;

for i = 2:length(N)
    p.Gp(1,i) = (R+B_tilde'*P*B_tilde)^(-1)*B_tilde'*X_tilde;
    X_tilde = Ac_tilde'*X_tilde;
end

p.stepCount = 1;
end