function output = plot3DRobot(opt_jointAngles,robot,param,trajectory)
close all;
%% Helper Functions
position = @(transform) transform(1:3,4);
%% Plot Robot Configurations
figure
init = zeros(param.numBodies,1)
show(robot,init);
view(2)
ax = gca;
ax.View = [0 0]
ax.Projection = 'orthographic';
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
center = zeros(4,param.numSamples);
supportFootPosition = zeros(3,param.numSamples);
swingFoot = zeros(3,param.numSamples);
for i = 1:length(opt_jointAngles)
    Htf = getTransform(robot,opt_jointAngles(:,i),'base',param.supportFoot)
    center(:,i) = Htf*[centerOfMass(robot,opt_jointAngles(:,i));1];
    supportFootPosition(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.supportFoot));
    swingFoot(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.swingFoot,param.supportFoot));
    p = plot3(center(1,i),center(2,i),center(3,i),'o');
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    drawnow
    waitfor(r);
end
%% Plot CoM position
n = size(opt_jointAngles,2);
figure(2)
subplot(3,1,1);
p1 = plot(1:length(opt_jointAngles),swingFoot(1,:),1:length(opt_jointAngles),trajectory(1,:));
p1(1).LineWidth = 5;
p1(2).LineWidth = 3;
legend('x_e(1) [m]','x^*_e(1) [m]');
xlabel('Sample [ΔT]');
ylabel('Position [m]');
title('End effector X position [m]');
subplot(3,1,2);
p2 = plot(1:length(opt_jointAngles),swingFoot(3,:),1:length(opt_jointAngles),trajectory(3,:));
p2(1).LineWidth = 5;
p2(2).LineWidth = 3;
ylim = [-0.05 0.05]; 
legend('x_e(3) [m]','x^*_e(3) [m]');
xlabel('Sample [ΔT]');
ylabel('Position [m]');
title('End effector Z position [m]');
subplot(3,1,3);
hold on;
plot(1:length(opt_jointAngles),ones(1,n)*0.075,'--');
hold on;
plot(1:length(opt_jointAngles),ones(1,n)*-0.075,'--');
hold on;
plot(1:length(opt_jointAngles),center(1,:),'LineWidth', 3);
legend('CoM upper limit [m] (support foot space)','CoM lower limit [m] (support foot space)','CoM position [m](support foot space)');
xlabel('Sample [ΔT]');
ylabel('Position [m]');
title('2D Quasi-static Constraints');

end

