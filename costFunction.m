function cost = costFuction(q,rSdPp,rCdWw,p,i)
%% Helper Functions
FK = Kinematics();
%% Cost
rSPp = FK.xe(q,p);
% Torso from world
Hwp = trvec2tform([p.footstep(p.stepCount,1)  (-1)^(p.stepCount+1)*0.055 -0.46])*roty(pi/2);        
% transform com from torso to support foot space
Htp = FK.Htp(q,p);
rCTt = FK.CoM(q,p);
rCPp =  Htp\[rCTt;1];
% transform com desired from world to support foot space
rCdPp = Hwp\[rCdWw(:,i);1];

cost = (rSdPp(:,i) - rSPp).'*(rSdPp(:,i) - rSPp) + ... % swing foot desired - swing foot
       (rCdPp - rCPp).'*(rCdPp - rCPp)+ ...
       0.5*trace(eye(3)-FK.R(q,p)) + ...
       0.5*trace(eye(3)-FK.RL(q,p));
%    + ...        0.5*trace(eye(3)-FK.RR(q,p));
%    + ...
%         + ...
end