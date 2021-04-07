function output = plot3DRobot(opt_jointAngles,robot,param)
close all;
%% Build Robot
% robot = importrobot('NUgus.urdf');
% robot.DataFormat = 'column';
%% Plot Robot Configurations
count = length(opt_jointAngles);
figure
init = zeros(param.N,1)
show(robot,init);
view(2)
ax = gca;
ax.Projection = 'perspective';
hold on
%convert trajectory to torso space
trajectory = [param.trajectory(1,:);param.trajectory(2,:);param.trajectory(3,:)];

framesPerSecond = 5;
r = rateControl(framesPerSecond);
%plot support polygon/convex hull
k = [0.115, 0.065;-0.115, 0.065;-0.115, -0.065;0.115, -0.065;0.115, 0.065;];
%% Initialize video
myVideo = VideoWriter('myVideoFile','MPEG-4'); %open video file
myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me
open(myVideo)
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'k')
plot3(k(:,1),k(:,2),zeros(5,1))
for i = 1:count
    center = centerOfMass(robot,opt_jointAngles(:,i));
    p = plot3(center(1),center(2),center(3),'o');
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    drawnow
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    waitfor(r);
end
close(myVideo)

end

