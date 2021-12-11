% Params
p = gaitParameters(robot);
g = p.g;        % Gravity
zc = p.zc;       % Center of Mass Height (constant)
Ts = p.Ts;       % Sample Time 
sim_time = length(p.footsteps) * p.step_time - p.t_preview - p.Ts;
% Step parameters
step_length_x = p.step_length_x;
step_width = p.step_width;
step_time = p.step_time;
% Define state space matrices
A = [0,1,0;0,0,1;0,0,0];
B = [0;0;1];
C = [1,0,-zc/g; 1,0,0; 0,1,0];% zmp, pos, vel
D = [0;0;0];
sys = ss(A,B,C,D);
% Discretize plant
sys_d = c2d(sys,Ts);
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;
% Assign measured outputs
sys_d = setmpcsignals(sys_d, 'MeasuredOutputs', 1, 'UnmeasuredOutputs',[2 3]);
% MPC parameters
predictionHorizon = 300;
numOutputs = 3;
initialState = [0;0;0];
% Create MPC controller object with sample time
mpc1 = mpc(sys_d, Ts);
%% specify prediction horizon
mpc1.PredictionHorizon = 300;
%% specify control horizon
mpc1.ControlHorizon = 300;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = [0;0;0];
%% specify weights
mpc1.Weights.MV = 0;
mpc1.Weights.MVRate = 0.1;
mpc1.Weights.OV = [1 0 0];
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'on';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%Simulate
init = [0;0;0];
sim('ZMP');
%% Plot
figure(1);
plot(zmp_ref_x(1,:),zmp_ref_y(1,:),com_x,com_y)
legend('zmp','CoM') 
figure(2);
hold on;
plot(tout,com_x,tout,com_y)
figure(3);
plot(tout,zmp_ref_y(1,:),tout,zmp_y,tout,com_y)
title('ZMP tracking Y');
legend('ZMP_y reference [m]','ZMP_y [m]','com_y [m]')
xlabel('time [s]') 
ylabel('y displacement [m]') 
figure(4);
plot(tout,zmp_ref_x(1,:),tout,zmp_x,tout,com_x)
title('ZMP tracking X');
legend('ZMP_x reference [m]','ZMP_x [m]','com_x [m]')
xlabel('time [s]') 
ylabel('x displacement [m]') 
com_trajectory = [com_x';com_y';zeros(1,length(com_x))];
figure(6);
hold on;
show(p.robot,p.initial_conditions);
plot3(com_x',com_y',zeros(1,length(com_x)),'LineWidth',3);


