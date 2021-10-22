function cost = costFunctionStatic(q,rSdPp,joints0,p,i)
    %% Helper Functions
    FK = Kinematics();
    %% Cost
    rSPp = FK.xe(q,p);
    %% Orientations
    Thetarl = rot2rpy(FK.R(q,p));
    Theta0r = rot2rpy(FK.RR(q,p)); 
    Theta0l = rot2rpy(FK.RL(q,p)); 
    
    % target orientation
    target_theta = [0 1.5708 0];
    Qd = eul2quat(target_theta);
    nd = Qd(1);
    Ed = Qd(2:4).';
    % current orientation
    
    Qe_r = eul2quat(Theta0r);
    ne_r = Qe_r(1);
    Ee_r = Qe_r(2:4).';
    
    Qe_l = eul2quat(Theta0l);
    ne_l = Qe_l(1);
    Ee_l = Qe_l(2:4).';

    K = [0.001,0,0;0,0.05,0;0,0,0.0005];
    skew = @(u) [0 -u(3) u(2);
                 u(3) 0 -u(1);
                 -u(2) u(1) 0];
             
    orientation_cost_right = ne_r*Ed - nd*Ee_r - skew(Ed)*Ee_r;
    orientation_cost_left = ne_l*Ed - nd*Ee_l - skew(Ed)*Ee_l;
    
    roty = @(theta) [cos(theta), 0,sin(theta); ...
              0,1,0; ...
              -sin(theta), 0, cos(theta);
              ];
             
    cost = norm(rSdPp(:,i) - FK.xe(q,p))  + ... % swing foot desired - swing foot
            + 0.5*trace(eye(3)-FK.R(q,p)) ...
            + 0.5*trace(eye(3)-roty(-pi/2)*FK.RL(q,p)) ...
            + 0.5*trace(eye(3)-roty(-pi/2)*FK.RR(q,p)) + ...
            0.0001*norm(joints0-q);
            
            %
            %Thetarl*Thetarl.' + ...
            %            + orientation_cost_right.'*orientation_cost_right + ...
            %            + orientation_cost_left.'*orientation_cost_left + ...
            %

end