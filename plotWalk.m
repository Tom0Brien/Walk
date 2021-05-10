function output = plotWalk(opt_jointAngles,robot,param,trajectory)
%% Plot Robot Configurations
init = zeros(param.numBodies,1);
show(robot,init);
view(2)
ax = gca;
ax.View = [45 45];
ax.Projection = 'perspective';
hold on
framesPerSecond = 20;
r = rateControl(framesPerSecond);
%% Initialize video
n = size(opt_jointAngles);
for i = 1:n(2)
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    X = [-1 1 1 -1];
    Y = [1 1 -1 -1];
    Z = [-0.5 -0.5 -0.5 -0.5];
    fill3(X,Y,Z,[211,211,211]/1000);
    axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    drawnow
    waitfor(r);
end
end

