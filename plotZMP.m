function rCTt = plotZMP(p,rCPp)
%% Plot ZMP and CoM trajectory

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

figure;
title('ZMP Trajectory Tracking')
title('X-Axis');
plot(p.com_x,'LineWidth',3.5);
hold on;
plot(rCWW(1,:),'--','LineWidth',3);
xlabel('Sample [Ts]')
ylabel('X [m]')
legend('Desired CoM [m] (Trajectory) ','Achieved CoM [m](IK)');

figure;
subplot(2,1,1)
title('ZMP X-Axis');
plot(p.zmp_x_star,'LineWidth',3);
hold on;
% plot(p.zmp_x,'LineWidth',3);
% plot(p.zmp_x_star+0.1,'--','LineWidth',3);
% plot(p.zmp_x_star-0.08,'--','LineWidth',3);
% plot(p.com_x,'LineWidth',3.5);
xlabel('Sample [Ts]')
ylabel('X [m]')
legend('Desired ZMP [m] (Trajectory)','Achieved ZMP(Preview Control)','Support polygon upper limit','Support Support polygon lower limit','CoM X');
xlabel('Sample [Ts]')
ylabel('x [m]')
xlim([0 360])

subplot(2,1,2)
title('ZMP Y-Axis');
plot(p.zmp_y_star,'LineWidth',3);
hold on;
% plot(p.zmp_y,'LineWidth',3);
% plot(p.zmp_y_star+0.05,'--','LineWidth',3);
% plot(p.zmp_y_star-0.05,'--','LineWidth',3);
% plot(p.com_y,'LineWidth',3.5);
xlabel('Sample [Ts]')
ylabel('Y [m]')
legend('Desired ZMP [m] (Trajectory)','Achieved ZMP(Preview Control)','Support polygon upper limit','Support Support polygon lower limit','CoM Y');
xlabel('Sample [Ts]')
ylabel('y [m]')
xlim([0 360])

figure;
title('Y-Axis');
plot(p.com_y,'LineWidth',3.5);
hold on;
plot(rCWW(2,:),'--','LineWidth',3);
xlabel('Sample [Ts]')
ylabel('Y [m]')
legend('Desired CoM [m] (Trajectory) ','Achieved CoM [m](IK)');


figure('name','ZMP vs CoM');
plot(p.zmp_x, p.zmp_y,'LineWidth',3);
plot(p.zmp_x_star, p.zmp_y_star,'LineWidth',3);
legend('ZMP_X [m]','ZMP_Y [m]');
xlabel('Sample [Ts]')
ylabel('X [m]')
hold on;
plot(p.com_x, p.com_y,'LineWidth',3);
plot(rCWW(1,1:3*p.N), rCWW(2,1:3*p.N),'--','LineWidth',1.5);
legend('ZMP* [m]','CoM* [m]','Achieved CoM [m]', 'ZMP');
xlabel('X [m]')
ylabel('Y [m]')

figure;

%% Helper
FK = Kinematics();
%% Plot Robot Configurations
init = zeros(p.num_bodies,1);
show(p.robot,init);
view(2)
ax = gca;
ax.View = [0 45];
ax.Projection = 'perspective';
hold on
framesPerSecond = 100;
r = rateControl(framesPerSecond);
hold on;
%% Initialize video
% t = torso, p = planted foot
Htp = FK.Htp(zeros(20,1),p);
% F = footsteps postion, P = planted foot
rPTt = FK.xs(zeros(20,1),p);
footwidth = 0.1;
footlenth = 0.2;
% fill floor
hold on;
floor_width = 2;
X = [-floor_width floor_width floor_width -floor_width];
Y = [floor_width floor_width -floor_width -floor_width];
Z = [rPTt(3)-0.001 rPTt(3)-0.001 rPTt(3)-0.001 rPTt(3)-0.001];
fill3(X,Y,Z,'white');
xlim([-floor_width floor_width])
ylim([-floor_width floor_width])
zlim([-0.5 1.2])
% plot footsteps
for i=1:size(p.footsteps,1)
    rFPp = p.footsteps(i,:).' + [0;0;rPTt(3)];
    hold on;
    X = [rFPp(1)-footlenth/2 rFPp(1)+footlenth/2 rFPp(1)+footlenth/2 rFPp(1)-footlenth/2];
    Y = [rFPp(2)+footwidth/2 rFPp(2)+footwidth/2 rFPp(2)-footwidth/2 rFPp(2)-footwidth/2];
    Z = [rPTt(3) rPTt(3) rPTt(3) rPTt(3)];
    fill3(X,Y,Z,'cyan');
end

hold on;
plot3(p.com_x, p.com_y,-0.495*zeros(1,size( p.com_y,2)),'LineWidth',5);
plot3(rCWW(1,:),rCWW(2,:),-0.495*zeros(1,size(rCWW,2)),'--','LineWidth',3);
plot3(p.zmp_x, p.zmp_y,-0.495*ones(1,size(p.zmp_y,2)),'LineWidth',3,'color','red');


end