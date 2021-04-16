function output = plot3DRobot(opt_jointAngles,robot,param)
close all;
%% Helper Functions
position = @(transform) transform(1:3,4);
rotation = @(transform) transform(1:3,1:3);
%% Plot Robot Configurations
figure
init = zeros(param.NumBodies,1)
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
myVideo = VideoWriter('myVideoFile','MPEG-4'); %open video file
myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me
open(myVideo)
trajectoryF = zeros(3,param.numSamples);
kF = zeros(3,param.numSamples);
for i = 1:param.numSamples
    center = centerOfMass(robot,opt_jointAngles(:,i));
    trajectoryF(:,i) = position(getTransform(robot,opt_jointAngles(:,i),param.supportFoot,'torso')) + rotation(getTransform(robot,opt_jointAngles(:,i),param.supportFoot,'torso'))*[param.trajectory(1,i);param.trajectory(2,i);param.trajectory(3,i)];
%     kF(:,i) = position(getTransform(robot,opt_jointAngles(:,i),'left_foot','torso')) + rotation(getTransform(robot,opt_jointAngles(:,i),'left_foot','torso'))*k;
    plot3(trajectoryF(1,i),trajectoryF(2,i),trajectoryF(3,i),'x');

    p = plot3(center(1),center(2),center(3),'o');
    show(robot,opt_jointAngles(:,i),'PreservePlot',false);
    drawnow
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    waitfor(r);
end
close(myVideo)

end

