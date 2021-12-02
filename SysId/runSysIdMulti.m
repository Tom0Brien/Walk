close all;
clc;
clear;

encoder_a = load('encoder_a.mat');
encoder_b = load('encoder_b.mat');

% Generate time vector (cut off first section)
cut_off = 250;
N = size(encoder_a.encoder);
t = 0.01*(1:N-cut_off);

% Extract encoder data wanted

encoder_extract_a = encoder_a.encoder(1+cut_off:N);
encoder_extract_b = encoder_b.encoder(1+cut_off:N);

figure;
grid on;
plot(encoder_extract_a, 'LineWidth',3)
xlabel('Sample [Ts]')
ylabel('Encoder Angle [rad]')
title('Encoder Measurements A')

figure;
grid on;
plot(encoder_extract_b, 'LineWidth',3)
xlabel('Sample [Ts]')
ylabel('Encoder Angle [rad]')
title('Encoder Measurements B')

%% Run optimisation
params0 = [0.131;0.069;0.00728437;0.01;encoder_extract_a(1,:);encoder_extract_b(1,:);0.01;0.01;0.01];
% Define optimisati on cost
constraint = @(params) nonlcon(params);
options = optimset('TolFun', 1e-6, 'TolCon', 1e-6);
cost = @(params) norm(vertcat(encoder_extract_a,encoder_extract_b) - pendSim(t,params,encoder_extract_a,encoder_extract_b));
% Run optimisation to find model parameters
[params_opt] = fmincon(cost,params0,[],[],[],[],[],[],constraint,options);
% Get our estimate angle
[recovered_theta] = pendSim(t,params_opt,encoder_extract_a,encoder_extract_b);


%% Plot
N = size(t,2);
figure;
plot(t, encoder_extract_a,'--','LineWidth',2);
hold on;
plot(t, recovered_theta(1:N),'--','LineWidth',2);
legend({'Encoder [rad]','Recovered Angle [rad]'})
xlabel('time [s]');
ylabel('Angle [rad]');
title('Swing Test A')

figure;
plot(t, encoder_extract_b,'--','LineWidth',2);
hold on;
plot(t, recovered_theta(N+1:end),'--','LineWidth',2);
legend({'Encoder [rad]','Recovered Angle [rad]'})
xlabel('time [s]');
ylabel('Angle [rad]');
title('Swing Test B')

disp('Estimated CoM from point A: ')
disp(params_opt(1))
disp('Estimated CoM from point B: ')
disp(params_opt(2))
disp('Estimated Inertia: ')
disp(params_opt(3))
disp('Estimated initial dtheta for point A: ')
disp(params_opt(5))
disp('Estimated initial dtheta for point B: ')
disp(params_opt(6))


%% Functions
function [theta] = pendSim(t_sim,params,encoder_extract_a,encoder_extract_b)
    % Unpack parameters
    la = params(1);
    lb = params(2);
    J = params(3);
    d_t_a = params(4);
    
    theta0_a = params(5);
    theta0_b = params(6);
    
    dtheta0_a = params(7);
    dtheta0_b = params(8);
    d_t_b = params(9);
    
    % Define known parameters
    mp = 0.3424;
    g = 9.81;
    % Define pendulum ODE's. x = (dtheta, theta)
    Ia = mp*la^2+J;
    Ib = mp*lb^2+J;
    ddtheta_a = @(x) -(d_t_a*x(1))/Ia - (mp*g*la*sin(x(2)))/Ia;
    ddtheta_b = @(x) -(d_t_b*x(1))/Ib - (mp*g*lb*sin(x(2)))/Ib;
    dtheta = @(x) x(1);
    
    dx_a = @(t,x) [ddtheta_a(x); dtheta(x)];
    dx_b = @(t,x) [ddtheta_b(x); dtheta(x)];
    % Define simulation initial conditionas and run simulation
    initial_a = [dtheta0_a, theta0_a]';
    initial_b = [dtheta0_b, theta0_b]';
    % Run simulation
    [~,x_a] = ode45(dx_a,t_sim,initial_a);
    [~,x_b] = ode45(dx_b,t_sim,initial_b);
    % Return simulated angles
    theta = vertcat(x_a(:,2),x_b(:,2));

end