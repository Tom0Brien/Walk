function [c,ceq] = nonlconFoot(joint_angles,robot,param,joints0)
    position = @(transform) transform(1:3,4);
    %% Convert centerOfMass into foot space
    rCTt = [centerOfMass(robot,joint_angles);1];
    %Keep CoM in support polygon (foot_space)
    Hft = getTransform(robot,joint_angles,'torso',param.supportFoot);
    rCFf = Hft*rCTt; %CoM to foot space
    %% Constraints
    c =[
        rCFf(1)-0.5;...  %CoMx <= 0.115
        -rCFf(1)-0.5;... %CoMx >= -0.115
        rCFf(2)-0.065;...  %CoMy <= 0.065
        -rCFf(2)-0.065;... %CoMx <= -0.065
        %Ensure knees are bent backwards
        joint_angles(15) - pi/2;
        -joint_angles(15);
        joint_angles(4) - pi/2;
        -joint_angles(4);
        %Ensure next solution is within 0.75 rads of previous
        norm(joint_angles(1) - joints0(1)) - 0.5;
        norm(joint_angles(2) - joints0(2)) - 0.5;
        norm(joint_angles(3) - joints0(3)) - 0.5;
        norm(joint_angles(4) - joints0(4)) - 0.5;
        norm(joint_angles(5) - joints0(5)) - 0.5;
        norm(joint_angles(6) - joints0(6)) - 0.5;
        norm(joint_angles(12) - joints0(1)) - 0.5;
        norm(joint_angles(13) - joints0(2)) - 0.5;
        norm(joint_angles(14) - joints0(3)) - 0.5;
        norm(joint_angles(15) - joints0(4)) - 0.5;
        norm(joint_angles(16) - joints0(5)) - 0.5;
        norm(joint_angles(17) - joints0(6)) - 0.5;
        norm(position(getTransform(robot,param.initialConditions,param.swingFoot,param.supportFoot)) - position(getTransform(robot,joint_angles,param.swingFoot,param.supportFoot))) - 0.01;
       ];
   ceq =[
         %Keep upper body rigid
         joint_angles(7);
         joint_angles(8);
         joint_angles(9);
         joint_angles(18);
         joint_angles(19);
         joint_angles(20);
         joint_angles(11);
         0;
         0;
         0;
        ];
end