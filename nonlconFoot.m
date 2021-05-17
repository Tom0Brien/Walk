function [c,ceq] = nonlconFoot(joint_angles,param,joints0)
    %% Constraints
    FK = kinematics3D(joint_angles,param);
    xs = FK.xs;
    %% Convert centerOfMass into foot space
    rCTt = [FK.com;1];
    rCFf = FK.Htf\rCTt; %CoM to foot space
    c =[
        rCFf(3)-0.115;...  %CoMx <= 0.115
        -rCFf(3)-0.115;... %CoMx >= -0.115
        rCFf(2)-0.065;...  %CoMy <= 0.065
        -rCFf(2)-0.065;... %CoMx <= -0.065
        %Ensure knees are bent backwards
        joint_angles(15) - pi/2;
        -joint_angles(15);
        joint_angles(4) - pi/2;
        -joint_angles(4);
        % Ensure support doesnt shift backwards
        -xs(1) - 0.1;
        %Ensure next solution is within 0.75 rads of previous
%         norm(joint_angles(1) - joints0(1)) - 1;
%         norm(joint_angles(2) - joints0(2)) - 1;
%         norm(joint_angles(3) - joints0(3)) - 1;
%         norm(joint_angles(4) - joints0(4)) - 1;
%         norm(joint_angles(5) - joints0(5)) - 1;
%         norm(joint_angles(6) - joints0(6)) - 1;
%         norm(joint_angles(12) - joints0(1)) - 1;
%         norm(joint_angles(13) - joints0(2)) - 1;
%         norm(joint_angles(14) - joints0(3)) - 1;
%         norm(joint_angles(15) - joints0(4)) - 1;
%         norm(joint_angles(16) - joints0(5)) - 1;
%         norm(joint_angles(17) - joints0(6)) - 1;
       ];

    left_hip_yaw = joint_angles(1);
    left_hip_pitch = joint_angles(3);
    left_hip_roll = joint_angles(2);

    right_hip_yaw = joint_angles(12);
    right_hip_pitch = joint_angles(14);
    right_hip_roll = joint_angles(13);

    if(param.supportFoot == "left_foot")
        hip = left_hip_pitch;
    else
        hip = right_hip_pitch;
    end
   ceq =[
         %Keep upper body rigid
         joint_angles(1);
         joint_angles(7);
         joint_angles(8);
         joint_angles(9);
         joint_angles(18);
         joint_angles(19);
         joint_angles(20);
         joint_angles(11);
         %Keep support foot hip rigid
         hip;
         0;
         0;
        ];
end