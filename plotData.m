function output = plotData(opt_jointAngles,robot,param,trajectory)
%% Helper Functions
position = @(transform) transform(1:3,4);
n = size(opt_jointAngles);
center = zeros(3,n(2));
centerSupport = zeros(4,n(2));
swingFootPosition = zeros(3,n(2));
for i = 1:n(2)
    center(:,i) = centerOfMass(robot,opt_jointAngles(:,i));
    swingFootPosition(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.swingFoot,param.supportFoot));
    Hft = getTransform(robot,opt_jointAngles(:,i),'torso',param.supportFoot);
    centerSupport(:,i) = Hft*[center(:,i);1];
end
%% Plot CoM position
figure
hold on;
plot(1:n(2),ones(1,n(2))*-0.115,'--');
hold on;
plot(1:n(2),ones(1,n(2))*0.115,'--');
hold on;
plot(1:n(2),centerSupport(1,:));
legend('CoM upper limit','CoM lower limit','CoM position (support foot space)');
title('x CoM constraint')
figure
hold on;
plot(1:n(2),ones(1,n(2))*0.13/2,'--');
hold on;
plot(1:n(2),ones(1,n(2))*-0.13/2,'--');
hold on;
plot(1:n(2),centerSupport(2,:));
legend('CoM upper limit','CoM lower limit','CoM position (support foot space)');
title('y CoM constraint')
%% Plot Trajectory vs End effector position
figure
plot(1:n(2),swingFootPosition(1,:),1:n(2),trajectory(1,:));
figure
plot(1:n(2),swingFootPosition(2,:),1:n(2),trajectory(2,:));
figure
plot(1:n(2),swingFootPosition(3,:),1:n(2),trajectory(3,:));
end

