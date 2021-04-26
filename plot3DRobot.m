function output = plot3DRobot(opt_jointAngles,robot,param)
close all;
%% Helper Functions
position = @(transform) transform(1:3,4);
%% Plot Robot Configurations
figure
init = zeros(param.numBodies,1)
show(robot,init);
view(2)
ax = gca;
ax.Projection = 'perspective';
hold on
%convert trajectory to torso space
framesPerSecond = 5;
r = rateControl(framesPerSecond);
%plot support polygon/convex hull
k = [0.115, 0.065;
    -0.115, 0.065;
    -0.115, -0.065;
    0.115, -0.065;
    0.115, 0.065;];
%% Initialize video
center = zeros(3,param.numSamples(2));
supportFootPosition = zeros(3,param.numSamples(2));
for i = 1:param.numSamples(2)
    center(:,i) = centerOfMass(robot,opt_jointAngles(:,i));
    supportFootPosition(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.supportFoot));
    p = plot3(center(1,i),center(2,i),center(3,i),'o');
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    drawnow
    waitfor(r);
end
%% Plot CoM position
figure(2)
hold on;
plot(1:param.numSamples(2),supportFootPosition(1,:)+0.115,'--');
hold on;
plot(1:param.numSamples(2),supportFootPosition(1,:)-0.115,'--');
hold on;
plot(1:param.numSamples(2),center(1,:));
legend('CoM upper limit','CoM lower limit','CoM position (torso space)');
figure(3)
hold on;
plot(1:param.numSamples(2),supportFootPosition(2,:)+0.13/2,'--');
hold on;
plot(1:param.numSamples(2),supportFootPosition(2,:)-0.13/2,'--');
hold on;
plot(1:param.numSamples(2),center(2,:));
legend('CoM upper limit','CoM lower limit','CoM position (torso space)');
end

