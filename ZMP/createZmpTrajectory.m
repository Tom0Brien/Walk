function [zmp_x,zmp_y] = createZmpTrajectory(footsteps, dt, t_step)
n_step = length(footsteps);
k = 1;
zmp_x = [];
zmp_y = [];
for i=0:dt:n_step*t_step
    zmp_x = [zmp_x footsteps(k,1)];
    zmp_y = [zmp_y footsteps(k,2)];
    if i ~=0 && mod(i,t_step) == 0
       k = k+1;
    end
end
end

