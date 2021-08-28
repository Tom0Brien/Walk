function [c,ceq] = nonlconFoot(q,param,joints0)
    %% Constraints
    FK = Kinematics();
    rTPt = FK.xs(q,param);
    %% Convert centerOfMass into foot space
    rCTt = [FK.CoM(q,param);1];
    rCPp = FK.Htp(q,param)\rCTt; %CoM to planted foot space
    c =[
        % Ensure rCTp is always within support foot support polygon
        rCPp(3)-0.115;  
        -rCPp(3)-0.115 
        rCPp(2)-0.065;
        -rCPp(2)-0.065;
        %Ensure knees are always bent backwards
        q(15) - pi/2;
        -q(15);
        q(4) - pi/2;
        -q(4);
        % Ensure support doesnt shift backwards to far
%          -rTPt(1) - 0.15;
         rTPt(3) + 0.42; % rTPt(3) <= -0.4
       ];
    ceq = [];
end