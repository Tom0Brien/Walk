function output = plotFootsteps(p)
%% Helper
FK = Kinematics();
%% Plot Robot Configurations
init = zeros(p.num_bodies,1);
show(p.robot,init);
view(2)
ax = gca;
ax.View = [45 45];
ax.Projection = 'perspective';
hold on
framesPerSecond = 100;
r = rateControl(framesPerSecond);
hold on;
%% Initialize video
% t = torso, p = planted foot
Htp = FK.Htp(zeros(20,1),p);
% F = footsteps postion, P = planted foot
rPTt = FK.xs(zeros(20,1),p);
footwidth = 0.1;
footlenth = 0.2;
for i=1:size(p.footsteps,1)
    rFPp = p.footsteps(i,:).' + [0;0;rPTt(3)];
    hold on;
    X = [rFPp(1)-footlenth/2 rFPp(1)+footlenth/2 rFPp(1)+footlenth/2 rFPp(1)-footlenth/2];
    Y = [rFPp(2)+footwidth/2 rFPp(2)+footwidth/2 rFPp(2)-footwidth/2 rFPp(2)-footwidth/2];
    Z = [rPTt(3) rPTt(3) rPTt(3) rPTt(3)];
    fill3(X,Y,Z,'blue','EdgeColor','black','LineWidth',1);
end
end

