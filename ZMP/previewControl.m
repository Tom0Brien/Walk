function [com_x, com_y, zmp_x, zmp_y] = previewControl(p_x, p_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gp)

% State for com x and y direction
x_x = [0;0;0];
x_y = [0;0;0];

% Variable for plotting
com_x = [];
com_y = [];
zmp_x = [];
zmp_y = [];

k = 1;
for ii=0:dt:t_calc
    % y = Cx
    % y = output zmp
    y_x = C_d * x_x;
    y_y = C_d * x_y;
    
    % calculate error 
    % target zmp - current zmp
    e_x = p_x(k) - y_x;
    e_y = p_y(k) - y_y;
    
    % preview horizon
    preview_x = 0;
    preview_y = 0;
    j = 1;
    for n=ii:dt:(ii+t_preview)
        preview_x = preview_x + Gp(j)*p_x(k+j);
        preview_y = preview_y + Gp(j)*p_y(k+j);
        j = j+1;
    end
    
    % pick signal input u
    u_x = -Gi*e_x - Gx*x_x - preview_x;
    u_y = -Gi*e_y - Gx*x_y - preview_y;
    
    % update state
    x_x = A_d*x_x + B_d*u_x;
    x_y = A_d*x_y + B_d*u_y;
    
    % zmp output
    zmp_x = [zmp_x y_x]
    zmp_y = [zmp_y y_y]
    
    % save current state to array
    com_x = [com_x x_x(1)];
    com_y = [com_y x_y(1)];

    k = k + 1;
end
end

