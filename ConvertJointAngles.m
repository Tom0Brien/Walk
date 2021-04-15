joint_angles = zeros(20,16);
joint_angles2 = zeros(20,16);

joint_angles(5,:) = -opt_joint_angles(2,:); %left_ankle_pitch
joint_angles(4,:) = -opt_joint_angles(3,:); %left_knee_pitch
joint_angles(3,:) = -opt_joint_angles(4,:); %left_leg_pitch
joint_angles(2,:) = -opt_joint_angles(5,:); %left_leg_roll
joint_angles(1,:) = -opt_joint_angles(6,:); %left_leg_yaw

joint_angles(12,:) = opt_joint_angles(8,:); %right_leg_yaw
joint_angles(13,:) = opt_joint_angles(9,:); %right_leg_roll
joint_angles(14,:) = opt_joint_angles(10,:); %right_leg_pitch
joint_angles(15,:) = opt_joint_angles(11,:); %right_knee_pitch
joint_angles(16,:) = opt_joint_angles(12,:); %right_ankle_pitch

joint_angles2(5,:) = opt_joint_angles2(12,:); %left_ankle_pitch
joint_angles2(4,:) = opt_joint_angles2(11,:); %left_knee_pitch
joint_angles2(3,:) = opt_joint_angles2(10,:); %left_leg_pitch
joint_angles2(2,:) = opt_joint_angles2(9,:); %left_leg_roll
joint_angles2(1,:) = opt_joint_angles2(8,:); %left_leg_yaw

joint_angles2(12,:) = -opt_joint_angles2(6,:); %right_leg_yaw
joint_angles2(13,:) = -opt_joint_angles2(5,:); %right_leg_roll
joint_angles2(14,:) = -opt_joint_angles2(4,:); %right_leg_pitch
joint_angles2(15,:) = -opt_joint_angles2(3,:); %right_knee_pitch
joint_angles2(16,:) = -opt_joint_angles2(2,:); %right_ankle_pitch


framesPerSecond = 5;
r = rateControl(framesPerSecond);
figure(3)
for i=1:16
    show(torsoRobot,joint_angles(:,i),'PreservePlot',false);
    drawnow
    waitfor(r);
end
for i=1:16
    show(torsoRobot,joint_angles2(:,i),'PreservePlot',false);
    drawnow
    waitfor(r);
end

