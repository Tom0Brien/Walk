function [c,ceq] = nonlconFoot(joint_angles,robot,param,joints0)
    FK = kinematics3D(joint_angles,param);
    
    %% Convert centerOfMass into foot space
    rCTt = [FK.com;1];
    rCFf = FK.Htf\rCTt; %CoM to foot space
    
    %% Constraints
    c =[
        %Ensure CoM (foot space) is within support polygon
        rCFf(3)-0.115;...  %CoMx <= 0.115
        -rCFf(3)-0.115;... %CoMx >= -0.115
        rCFf(2)-0.065;...  %CoMy <= 0.065
        -rCFf(2)-0.065;... %CoMx <= -0.065
        %Ensure knees are bent backwards
        joint_angles(15) - pi/2;
        -joint_angles(15);
        joint_angles(4) - pi/2;
        -joint_angles(4);
        %Ensure next solution is within 0.75 rads of previous
        norm(joint_angles(1) - joints0(1)) - 0.1;
        norm(joint_angles(2) - joints0(2)) - 0.1;
        norm(joint_angles(3) - joints0(3)) - 0.1;
        norm(joint_angles(4) - joints0(4)) - 0.1;
        norm(joint_angles(5) - joints0(5)) - 0.1;
        norm(joint_angles(6) - joints0(6)) - 0.1;
        norm(joint_angles(12) - joints0(1)) - 0.1;
        norm(joint_angles(13) - joints0(2)) - 0.1;
        norm(joint_angles(14) - joints0(3)) - 0.1;
        norm(joint_angles(15) - joints0(4)) - 0.1;
        norm(joint_angles(16) - joints0(5)) - 0.1;
        norm(joint_angles(17) - joints0(6)) - 0.1;
        -norm(joint_angles(1) - joints0(1)) - 0.1;
        -norm(joint_angles(2) - joints0(2)) - 0.1;
        -norm(joint_angles(3) - joints0(3)) - 0.1;
        -norm(joint_angles(4) - joints0(4)) - 0.1;
        -norm(joint_angles(5) - joints0(5)) - 0.1;
        -norm(joint_angles(6) - joints0(6)) - 0.1;
        -norm(joint_angles(12) - joints0(1)) - 0.1;
        -norm(joint_angles(13) - joints0(2)) - 0.1;
        -norm(joint_angles(14) - joints0(3)) - 0.1;
        -norm(joint_angles(15) - joints0(4)) - 0.1;
        -norm(joint_angles(16) - joints0(5)) - 0.1;
        -norm(joint_angles(17) - joints0(6)) - 0.1;
        %Ensure feet are in feasible region
%         norm([FK.xs(1);FK.xs(2);0]) - 0.15;
%         -norm([FK.xs(1);FK.xs(2);0]) - 0.1;
%         norm([FK.xe(1);FK.xe(2);0]) - 0.15;
%         -norm([FK.xe(1);FK.xe(2);0]) - 0.1;
       ];

    left_hip_yaw = joint_angles(1);
    left_hip_pitch = joint_angles(3);
    left_hip_roll = joint_angles(2);

    right_hip_yaw = joint_angles(12);
    right_hip_pitch = joint_angles(14);
    right_hip_roll = joint_angles(13);
    
    if(param.supportFoot == "left_foot")
        pitch = left_hip_pitch;
        roll = left_hip_roll;
        yaw = left_hip_yaw;
        knee = joint_angles(4);
    else
        pitch = right_hip_pitch;
        roll = right_hip_roll;
        yaw = right_hip_yaw;
        knee = joint_angles(15);
    end

   ceq =[
         %Keep upper body rigid
         joint_angles(7)-param.initialConditions(7);
         joint_angles(8)-param.initialConditions(8);
         joint_angles(9)-param.initialConditions(9);
         joint_angles(18)-param.initialConditions(18);
         joint_angles(19)-param.initialConditions(19);
         joint_angles(20)-param.initialConditions(20);
         joint_angles(11)-param.initialConditions(11);
         joint_angles(10)-param.initialConditions(10);
%          %Keep support foot hip rigid
         pitch;
         roll
%          knee;
         0;
        ];
end