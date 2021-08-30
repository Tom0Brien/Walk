function output = plotWalk(opt_jointAngles,robot,param)
%% Helper
CoM = @(kinematics) kinematics.com;
%% Plot Robot Configurations
init = zeros(param.num_bodies,1);
show(robot,init);
view(2)
ax = gca;
ax.View = [45 45];
ax.Projection = 'perspective';
hold on
framesPerSecond = 100;
r = rateControl(framesPerSecond);
%% Initialize video
n = size(opt_jointAngles,2)
center = zeros(3,n);
for i = 1:n
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    center(:,i) = CoM(kinematics3D(opt_jointAngles(:,i),param));
    hold on;
    plot3(center(1,i),center(2,i),center(3,i),'o');
    axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    drawnow
    waitfor(r);
end
end

