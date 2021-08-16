function H = trany(y)
    H = [[eye(3);zeros(1,3)],[0;y;0;1]];
end