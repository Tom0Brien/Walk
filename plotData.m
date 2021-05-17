function output = plotData(opt_jointAngles,param,trajectory)
%% Helper Functions
com = @(kinematics) kinematics.com;
xe  = @(kinematics) kinematics.xe;
Hft = @(kinematics) inv(kinematics.Htf);
%% Convert centerOfMass into foot space
n = size(opt_jointAngles,2);
center = zeros(3,n);
rCPp = zeros(4,n);
rSPp = zeros(3,n);
for i = 1:n
    center(:,i) = com(kinematics3D(opt_jointAngles(:,i),param));
    rSPp(:,i) = xe(kinematics3D(opt_jointAngles(:,i),param));
    Tft = Hft(kinematics3D(opt_jointAngles(:,i),param));
    rCPp(:,i) = Tft*[center(:,i);1];
end
%% Plot CoM position
figure
subplot(2,1,1);
hold on;
plot(1:n,ones(1,n)*-0.115,'--');
hold on;
plot(1:n,ones(1,n)*0.115,'--');
hold on;
plot(1:n,rCPp(3,:));
legend('CoM upper limit [m]','CoM lower limit [m]','CoM position [m] (support foot space)');
title('x CoM constraint')
subplot(2,1,2);
hold on;
plot(1:n,ones(1,n)*0.13/2,'--');
hold on;
plot(1:n,ones(1,n)*-0.13/2,'--');
hold on;
plot(1:n,rCPp(2,:));
legend('CoM upper limit [m]','CoM lower limit [m]','CoM position [m] (support foot space)');
title('y CoM constraint')
%% Plot Trajectory vs End effector position
figure
subplot(3,1,1);
plot(1:n,rSPp(1,:),1:n,trajectory(1,:));
legend('Swing Foot x Position  [m] (support foot space)','Swing Foot x Trajectory [m] (support foot space)');
subplot(3,1,2);
plot(1:n,rSPp(2,:),1:n,trajectory(2,:));
legend('Swing Foot y Position [m] (support foot space)','Swing Foot y Trajectory [m] (support foot space)');
subplot(3,1,3);
plot(1:n,rSPp(3,:),1:n,trajectory(3,:));
legend('Swing Foot z Position [m] (support foot space)','Swing Foot z Trajectory [m] (support foot space)');
end

