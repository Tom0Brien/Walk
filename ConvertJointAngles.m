joint_angles = zeros(20,30);
init = zeros(20,1)
for i = 1:30
joint_angles(:,i) = init;
end

% joint_angles(6,:) = -opt_joint_angles(1,:); not sure
joint_angles(5,:) = -opt_joint_angles(2,:); %minus
joint_angles(4,:) = -opt_joint_angles(3,:); % minus
joint_angles(3,:) = -opt_joint_angles(4,:); %minus
joint_angles(2,:) = opt_joint_angles(5,:); %minus
joint_angles(1,:) = -opt_joint_angles(6,:);

joint_angles(12,:) = opt_joint_angles(8,:);
joint_angles(13,:) = opt_joint_angles(9,:);
joint_angles(14,:) = opt_joint_angles(10,:);
joint_angles(15,:) = opt_joint_angles(11,:);
joint_angles(16,:) = opt_joint_angles(12,:);
joint_angles(17,:) = opt_joint_angles(13,:);
framesPerSecond = 5;
r = rateControl(framesPerSecond);
figure(3)
for i=1:30
    show(torsoRobot,joint_angles(:,i),'PreservePlot',false);
    drawnow
    waitfor(r);
end

