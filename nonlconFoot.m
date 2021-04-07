function [c,ceq] = nonlconFoot(joint_angles,robot)
    initialCondition = homeConfiguration(robot);
    CoM = centerOfMass(robot,joint_angles);
    %Keep CoM in support polygon (torso_space)
    c =[CoM(1)-0.21/2;...  %CoMx <= 0.115
        -CoM(1)-0.21/2;... %CoMx >= -0.115
        CoM(2)-0.13/2;...  %CoMy <= 0.065
        -CoM(2)-0.13/2;... %CoMx <= -0.065
       ];
   ceq =[
         0;... %Keep right foot roll fixed
         0;... %Keep right_ankle_pitch fixed
         0;... %Keep left_foot_pitch fixed
         0;
        ];
%    initJointAngles = initialConditions;
%     ceq = [
%            norm(Kinematics3D(joint_angles).leftPosition - initPos); %Keep left foot planted
%            norm(joint_angles(10)-initJointAngles(10));... %Keep neck fixed
%            norm(joint_angles(12)-initJointAngles(12));... %Keep right hip yaw fixed
%            norm(joint_angles(2) -initJointAngles(2));...  %Keep left hip roll fixed
%            norm(joint_angles(17)-initJointAngles(17));... %Keep right foot roll fixed
%            norm(joint_angles(5)- initJointAngles(5));...  %Keep left_ankle_pitch fixed
 %           norm(joint_angles(19)- initJointAngles(19));   %Keep right_shoulder_roll fixed
%            norm(joint_angles(16)- initJointAngles(16));   %Keep right_ankle_pitch fixed
%           ];
end