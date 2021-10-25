close all;
result = load('controllers/walk_controller/result.mat');
Hwp = trvec2tform([p.footsteps(1,1)  p.footsteps(1,2)  -0.46])*roty(pi/2);
rCWW = zeros(4,p.N);
for i = 1:p.N
    rCWW(:,i) = Hwp*rCPp(:,i);
end
Hwp = trvec2tform([p.footsteps(2,1)  p.footsteps(2,2) -0.46])*roty(pi/2);
for i = p.N+1:2*p.N
    rCWW(:,i) = Hwp*rCPp(:,i);
end
Hwp = trvec2tform([p.footsteps(3,1)  p.footsteps(3,2)  -0.46])*roty(pi/2);
for i = 2*p.N+1:3*p.N
    rCWW(:,i) = Hwp*rCPp(:,i);
end




%% COM
figure('name','CoM');
plot(rCWW(1,:),rCWW(2,:),'LineWidth',3);
hold on;
plot(result.com_position(1:3*p.N,1),result.com_position(1:3*p.N,2),'LineWidth',3);
legend('CoM Matlab [m]','CoM Webots [m]');
xlabel('X [m]')
ylabel('Y [m]')


%% Velocity
figure('name','Velocity X');
desired_velocity_x = (0.5*p.step_length_x/p.step_time)*ones(3*p.N);
plot(desired_velocity_x,'LineWidth',2);
hold on;
plot(result.velocity(1:3*p.N,1),'--','LineWidth',1.5);
legend('desired x velocity [m/s]', 'simulation x velocity [m/s]');
xlabel('Sample Ts')
ylabel('x velocity [m/s]')