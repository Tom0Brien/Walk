function output = plot3DRobot(opt_jointAngles,robot,param,trajectory)
close all;
%% Helper Functions
position = @(transform) transform(1:3,4);
%% Plot Robot Configurations
figure
init = zeros(param.numBodies,1);
show(robot,init);
view(2)
ax = gca;
ax.View = [45 45];
ax.Projection = 'perspective';
hold on
framesPerSecond = 2;
r = rateControl(framesPerSecond);
%% Initialize video
n = size(opt_jointAngles);
center = zeros(3,n(2));
swingFootPosition = zeros(3,n(2));
supportFootPosition = zeros(3,n(2));
for i = 1:n(2)
    center(:,i) = centerOfMass(robot,opt_jointAngles(:,i));
    swingFootPosition(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.swingFoot,param.supportFoot));
    supportFootPosition(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.supportFoot))
    plot3(center(1,i),center(2,i),center(3,i),'o');
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    Hft = getTransform(robot,opt_jointAngles(:,i),'torso',param.supportFoot);
    centerSupport(:,i) = Hft*[center(:,i);1]
    drawnow
    waitfor(r);
end
%% Plot CoM position
figure(2)
hold on;
plot(1:n(2),ones(1,n(2))*-0.115,'--');
hold on;
plot(1:n(2),ones(1,n(2))*0.115,'--');
hold on;
plot(1:n(2),centerSupport(1,:));
legend('CoM upper limit','CoM lower limit','CoM position (support foot space)');
title('x CoM constraint')
figure(3)
hold on;
plot(1:n(2),ones(1,n(2))*0.13/2,'--');
hold on;
plot(1:n(2),ones(1,n(2))*-0.13/2,'--');
hold on;
plot(1:n(2),centerSupport(2,:));
legend('CoM upper limit','CoM lower limit','CoM position (support foot space)');
title('y CoM constraint')
%% Plot Trajectory vs End effector position
figure(4)
plot(1:n(2),swingFootPosition(1,:),1:n(2),trajectory(1,:));
figure(5)
plot(1:n(2),swingFootPosition(2,:),1:n(2),trajectory(2,:));
figure(6)
plot(1:n(2),swingFootPosition(3,:),1:n(2),trajectory(3,:));

end

