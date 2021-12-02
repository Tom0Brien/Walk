function [c,ceq] = nonlconFoot(joint_angles,robot,param,joints0)
    %% Helper Functions
    position = @(transform) transform(1:3,4);
    rotation = @(transform) transform(1:3,1:3);
    %% Convert centerOfMass into foot space
    rCTt = centerOfMass(robot,joint_angles);
    %Keep CoM in support polygon (foot_space)
    RFTf = rotation(getTransform(robot,joint_angles,'base',param.supportFoot));
    rTFf = position(getTransform(robot,joint_angles,'base',param.supportFoot));
    rCFf = rTFf + RFTf*rCTt; %CoM to foot space
    rFTt = position(getTransform(robot,joint_angles,param.supportFoot));
    
    %% Keep CoM in Support Polygon
    c =[
%         -rFTt(1)-param.stepLength/2;
        rCFf(1)-0.075;...  %CoMx <= 0.115
        -rCFf(1)-0.075;... %CoMx >= -0.115
        %Ensure next solution isn't greater than 0.75 rads away
        norm(joint_angles(1) - joints0(1)) - 0.75;
        norm(joint_angles(2) - joints0(2)) - 0.75;
        norm(joint_angles(3) - joints0(3)) - 0.75;
        norm(joint_angles(4) - joints0(4)) - 0.75;
        norm(joint_angles(5) - joints0(5)) - 0.75;
        norm(joint_angles(6) - joints0(6)) - 0.75;
        %Ensure knees are bent backwards
        joint_angles(5) - pi/2;
        -joint_angles(5);
        joint_angles(2) - pi/2;
        -joint_angles(2);
       ];
   ceq =[
         0;...
         0;...
         0;
        ];
end