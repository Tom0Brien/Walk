function [c,ceq] = nonlcon(params)
    %% Constraints
    length_between_points = 0.2;
    c = [];
    ceq = [params(1)+params(2) - length_between_points];
end