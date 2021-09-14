function [c,ceq] = nonlconZMP(q)
    %% Constraints
    c =[
        %Ensure knees are bent backwards
        q(15) - pi/2;
        -q(15);
        q(4) - pi/2;
        -q(4);
        ];
    ceq = [q(12);q(1)];
end