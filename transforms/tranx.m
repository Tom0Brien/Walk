function H = tranx(x)
    H = [[eye(3);zeros(1,3)],[x;0;0;1]];
end