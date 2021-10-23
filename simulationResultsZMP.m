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
plot(rCWW(1,1:3*p.N),'--','LineWidth',3)
legend('ZMP_X [m]','CoM* X [m]','Actual CoM X [m]');
xlabel('Sample [Ts]')
ylabel('X [m]')


%% ZMP X-AXIS
figure('name','ZMP X-Axis');
plot(p.zmp_x,'LineWidth',5);
hold on;
plot(p.com_x,'LineWidth',5);
xlabel('Sample [Ts]')
ylabel('X [m]')

%% ZMP Y-AXIS
figure('name','ZMP Y-Axis');
plot(p.zmp_y,'LineWidth',3);
hold;
plot(p.com_y,'LineWidth',3);
plot(rCWW(2,1:3*p.N),'--','LineWidth',1.5)
legend('ZMP_Y [m]','CoM Y [m]','Actual CoM Y [m]');
xlabel('Sample [Ts]')
ylabel('Y [m]')

%% ZMP AND COM
figure('name','ZMP vs CoM');
plot(p.zmp_x, p.zmp_y,'LineWidth',3);
legend('ZMP_X [m]','ZMP_Y [m]');
xlabel('Sample [Ts]')
ylabel('X [m]')
hold on;
plot(p.com_x, p.com_y,'LineWidth',3);
plot(rCWW(1,1:3*p.N), rCWW(2,1:3*p.N),'--','LineWidth',1.5);
plot(result.com_position(1:3*p.N,1)-result.com_position(3,1),result.com_position(1:3*p.N,2)-result.com_position(3,2),'LineWidth',1.5);
legend('ZMP [m]','CoM [m]','Actual CoM [m]');
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