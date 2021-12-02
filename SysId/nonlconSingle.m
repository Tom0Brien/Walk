function [c,ceq] = nonlconSingle(params)
    %% Constraints
    c = [-params(5);-params(2);-params(1)];
    ceq = [];
end