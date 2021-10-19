function cost = costFuction(q,rSdPp,rCdWw,p,i)
    %% Helper Functions
    FK = Kinematics();
    %% Cost
    rSPp = FK.xe(q,p);
    % Torso from world
    Hwp = trvec2tform([p.footsteps(p.step_count,1) p.footsteps(p.step_count,2) -0.495])*roty(pi/2);        
    % transform com from torso to support foot space
    Htp = FK.Htp(q,p);
    rCTt = FK.CoM(q,p);
    rCPp =  (Htp\[rCTt;1]);
    % transform com desired from world to support foot space
    rCdPp = (Hwp\[rCdWw(:,i);1]);
    
    Thetarl = rot2rpy(FK.R(q,p));
    
    Theta0r = rot2rpy(FK.RR(q,p)); 
    Theta0l = rot2rpy(FK.RL(q,p)); 
    
    target_theta = [0 1.5708 0];

    K = [0.001,0,0;0,0.05,0;0,0,0.0005];
    cost = (rSdPp(:,i) - rSPp).'*(rSdPp(:,i) - rSPp) + ... % swing foot desired - swing foot
           (rCdPp - rCPp).'*(rCdPp - rCPp) + ...
            Thetarl*Thetarl.' + ...
            (Theta0r - target_theta)*K*(Theta0r - target_theta).' + ...
            (Theta0l - target_theta)*K*(Theta0l - target_theta).';
%        + ...
%            0.5*trace(eye(3)-FK.R(q,p))               + ...
%            0.5*trace(eye(3)-FK.RR(q,p))              + ...
%            0.5*trace(eye(3)-FK.RL(q,p));
%                         + ...
%          0.001*norm(YAW(FK.R(q,p)))  + 0.001*norm(PITCH(FK.R(q,p)))  + 0.00*norm(ROLL(FK.R(q,p))) + ...
%          0.001*norm(YAW(FK.RL(q,p))) + 0.001*norm(PITCH(FK.RL(q,p))) + 0.00*norm(ROLL(FK.RL(q,p))) + ...
%          0.001*norm(YAW(FK.RR(q,p))) + 0.001*norm(PITCH(FK.RR(q,p))) + 0.00*norm(ROLL(FK.RR(q,p)));

end