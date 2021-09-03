function output = plotWalk(opt_jointAngles,robot,p)
%% Helper
FK = Kinematics();
%% Plot Robot Configurations
init = zeros(p.num_bodies,1);
show(robot,init);
view(2)
ax = gca;
ax.View = [45 45];
ax.Projection = 'perspective';
hold on
framesPerSecond = 100;
r = rateControl(framesPerSecond);
%% Initialize video
n = size(opt_jointAngles,2);
rCTt = zeros(3,n);
for i = 1:n
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    rCTt(:,i) = FK.CoM(opt_jointAngles(:,i),p);
    hold on;
    plot3(rCTt(1,i),rCTt(2,i),rCTt(3,i),'o');
    axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    drawnow
    waitfor(r);
end
end

