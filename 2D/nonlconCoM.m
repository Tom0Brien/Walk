function [c,ceq] = nonlconFoot(joint_angles,robot,param,joints0)
    %% Convert centerOfMass into foot space
    position = @(transform) transform(1:3,4);
    %% Constraints
    c =[
        %Ensure knees are bent backwards
        joint_angles(5) - pi/2;
        -joint_angles(5);
        joint_angles(2) - pi/2;
        -joint_angles(2);
        %Ensure next solution is within 0.75 rads of previous
        norm(position(getTransform(robot,param.initialConditions,param.swingFoot,param.supportFoot)) - position(getTransform(robot,joint_angles,param.swingFoot,param.supportFoot)))-0.01;
       ];
   ceq =[
         %Keep upper body rigid
         0;
         0;
         0;
         0;
        ];
end