close all;
clc;
clear;

encoder = load('encoder_a.mat');

% Generate time vector (cut off first section)
cut_off = 150;
N = size(encoder.encoder);
t = 0.01*(1:N-cut_off);

% Extract encoder data wanted

encoder_extract = encoder.encoder(1+cut_off:N);
%encoder_measurements = encoder_measurements(0:N);

figure(2);
grid on;
plot(encoder_extract, 'LineWidth',3)
xlabel('Sample [Ts]')
ylabel('Encoder Angle [rad]')
title('Encoder Measurements')

% Define optimisation initial conditions
params0 = [0.01;0.0001;encoder_extract(1,:);0.001;0.00009];
% Define optimisation cost
cost = @(params) norm(encoder_extract - pendSim(t,params,encoder_extract));
% Run optimisation to find model parameters
nonlcon = @(params) nonlconSingle(params);
[params_opt, costVal] = fmincon(cost,params0,[],[],[],[],[],[],nonlcon);

% Get our estimate angle
[recovered_theta] = pendSim(t,params_opt,encoder_extract);

%% cad syd id

% Define optimisation initial conditions
% params0 = [0.1;0.1;0.1];
% % Define optimisation cost
% cost = @(params) norm(encoder_extract - cadPendSim(t,params,encoder_extract));
% % Run optimisation to find model parameters
% [params_opt, costVal] = fmincon(cost,params0);
% 
% [recovered_theta] = cadPendSim(t,params_opt,encoder_extract);

%% Multiple data set

% constraint = @(params) nonlcon(params);
%     
% cost = @(params) norm(encoder_extract - cadPendSim(t,params,encoder_extract));


%% Plot
figure(3)
plot(t, encoder_extract,'LineWidth',4);
hold on
grid on
plot(t, recovered_theta,'--','LineWidth',4);
legend({'Encoder [rad]','Recovered Angle [rad]'})
xlabel('time [s]');
ylabel('Angle [rad]');

%% Disp optimized params
disp('Estimated Length : ')
disp(params_opt(1))


%% Functions
function [theta] = pendSim(t_sim,params,encoder_extract)
    % Unpack parameters
    l = params(1);
    d_t = params(2);
    theta0 = params(3);
    dtheta0 = params(4);
    J = params(5);
    % Define known parameters
    mp = 0.1298/2;
    g = 9.81;
    
    % TODO: Define pendulum ODE. x = (dtheta, theta)
    I = mp*l^2+J;
    ddtheta = @(x) -(d_t*x(1))/I - (mp*g*l*sin(x(2)))/I;
    dtheta = @(x) x(1);
    dx = @(t,x) [ddtheta(x); dtheta(x)];
    
    % Define simulation initial conditionas and run simulation
    initial = [dtheta0, theta0]';
    % Run simulation
    [~,x] = ode45(dx,t_sim,initial);
    % Return simulated angles
    theta = x(:,2);
end

function [theta] = cadPendSim(t_sim,params,encoder_extract)
    % Unpack parameters
    d_t = params(1);
    l = params(2);
    theta0 = encoder_extract(1,:);
    dtheta0 = params(3);
    % Define known parameters
    mp = 0.3424;
    g = 9.81;
    I = 0.00728437;
    
    % TODO: Define pendulum ODE. x = (dtheta, theta)
    ddtheta = @(x) -d_t*x(1)/I - mp*g*l*sin(x(2))/I;
    dtheta = @(x) x(1);
    dx = @(t,x) [ddtheta(x); dtheta(x)];
    
    % TODO: Define simulation initial conditionas and run simulation
    initial = [dtheta0, theta0]';
    % Run simulation
    [~,x] = ode45(dx,t_sim,initial);
    % TODO: Return simulated angles
    theta = x(:,2);
%     dtheta = x(:,1);
end

function [theta] = multiPendSim(t_sim,params,encoder_extract)
    % Unpack parameters
    d_t = params(1);
    l = params(2);
    theta0 = encoder_extract(1,:);
    dtheta0 = params(3);
    % Define known parameters
    mp = 0.3424;
    g = 9.81;
    I = 0.00728437;
    
    % TODO: Define pendulum ODE. x = (dtheta, theta)
    ddtheta = @(x) -d_t*x(1)/I - mp*g*l*sin(x(2))/I;
    dtheta = @(x) x(1);
    dx = @(t,x) [ddtheta(x); dtheta(x)];
    
    % TODO: Define simulation initial conditionas and run simulation
    initial = [dtheta0, theta0]';
    % Run simulation
    [~,x] = ode45(dx,t_sim,initial);
    % TODO: Return simulated angles
    theta = x(:,2);
%     dtheta = x(:,1);
end