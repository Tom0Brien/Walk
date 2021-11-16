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


%% ZMP X-AXIS
figure('name','ZMP X-Axis');
subplot(3,1,1);
hold on
% plot(p.zmp_x,'LineWidth',5);
hold on;
plot(p.com_x,'LineWidth',5);
xlabel('Sample [Ts]')
ylabel('X [m]')
plot(rCWW(1,1:3*p.N),'--','LineWidth',3)
plot(result.com_position(1:3*p.N,1)-result.com_position(3,1),'--','LineWidth',3)
legend('CoM* X [m]','Achieved CoM X [m] (Matlab)','Achieved CoM X [m] (Simulation)');
xlabel('Sample [Ts]')
ylabel('X [m]')

%% ZMP Y-AXIS
subplot(3,1,2);
% plot(p.zmp_y,'LineWidth',3);
hold on;
plot(p.com_y,'LineWidth',5);
plot(rCWW(2,1:3*p.N),'--','LineWidth',3)
plot(result.com_position(1:3*p.N,2)-result.com_position(3,2),'--','LineWidth',3)
legend('CoM Y [m]','Achieved CoM Y [m] (Matlab)','Achieved CoM Y [m] (Simulation)');
xlabel('Sample [Ts]')
ylabel('Y [m]')

%% ZMP AND COM
subplot(3,1,3);
FK = Kinematics();
rPTt = FK.xs(zeros(20,1),p);
footwidth = 0.1;
footlenth = 0.2;
for i=1:3
    rFPp = p.footsteps(i,:).' + [0;0;rPTt(3)];
    hold on;
    X = [rFPp(1)-footlenth/2 rFPp(1)+footlenth/2 rFPp(1)+footlenth/2 rFPp(1)-footlenth/2];
    Y = [rFPp(2)+footwidth/2 rFPp(2)+footwidth/2 rFPp(2)-footwidth/2 rFPp(2)-footwidth/2];
    fill(X,Y,'white');
end
xlabel('Sample [Ts]')
ylabel('X [m]')
hold on;
plot(p.com_x, p.com_y,'LineWidth',5);
plot(rCWW(1,1:3*p.N), rCWW(2,1:3*p.N),'--','LineWidth',3);
plot(result.com_position(3:3*p.N,1)-result.com_position(3,1),result.com_position(3:3*p.N,2)-result.com_position(3,2),'LineWidth',3);
legend('Foot Step', 'Foot Step', 'Foot Step', 'Desired CoM [m]','Achieved CoM [m] (Matlab)','Achieved CoM [m] (Simulation)');
xlim([-0.1 0.5]);
ylim([-0.15 0.15]);
xlabel('X [m]')
ylabel('Y [m]')

figure;
for i=1:3
    rFPp = p.footsteps(i,:).' + [0;0;rPTt(3)];
    hold on;
    X = [rFPp(1)-footlenth/2 rFPp(1)+footlenth/2 rFPp(1)+footlenth/2 rFPp(1)-footlenth/2];
    Y = [rFPp(2)+footwidth/2 rFPp(2)+footwidth/2 rFPp(2)-footwidth/2 rFPp(2)-footwidth/2];
    fill(X,Y,'white');
end
xlabel('Sample [Ts]')
ylabel('X [m]')
hold on;
title('CoM Trajectory ZMP');
plot(p.com_x, p.com_y,'LineWidth',5);
plot(rCWW(1,1:3*p.N), rCWW(2,1:3*p.N),'--','LineWidth',3);
plot(result.com_position(3:3*p.N,1)-result.com_position(3,1),result.com_position(3:3*p.N,2)-result.com_position(3,2),'LineWidth',3);
legend('Foot Step', 'Foot Step', 'Foot Step', 'Desired CoM [m]','Achieved CoM [m] (Matlab)','Achieved CoM [m] (Simulation)');
xlim([-0.1 0.5]);
ylim([-0.15 0.15]);
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