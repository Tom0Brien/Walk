function [c,ceq] = nonlconFoot(joint_angles,robot,param)
    %% Helper Functions
    position = @(transform) transform(1:3,4);
    rotation = @(transform) transform(1:3,1:3);
    %% Convert centerOfMass into foot space
    rct = centerOfMass(robot,joint_angles);
    %Keep CoM in support polygon (foot_space)
    Rft = rotation(getTransform(robot,joint_angles,param.supportFoot));
    rtf = position(getTransform(robot,joint_angles,param.supportFoot,'torso'));
    rcf = rtf + Rft*rct; %CoM to foot space
    %% Keep CoM in Support Polygon
    c =[
        rcf(1)-0.21/2;...  %CoMx <= 0.115
        -rcf(1)-0.21/2;... %CoMx >= -0.115
        rcf(2)-0.13/2;...  %CoMy <= 0.065
        -rcf(2)-0.13/2;... %CoMx <= -0.065
       ];
   ceq =[
         0;... 
         0;...
         0;...
         0;
        ];
end