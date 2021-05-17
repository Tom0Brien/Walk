function output = plotCoM(joint_angles,param)
%% Helper Functions
position = @(transform) transform(1:3,4);
Ry = @(theta)[cos(theta), 0,sin(theta),0; ...
              0,1,0,0; ...
              -sin(theta), 0, cos(theta),0;...
              0,0,0,1
              ];
Hft = @(kinematics) inv(kinematics.Htf);
%% Plot Robot Configurations
figure
robot = param.robot;
framesPerSecond = 30;
r = rateControl(framesPerSecond);
%% Initialize video
n = size(joint_angles);
center = zeros(4,n(2));
foot = zeros(3,n(2));
for i = 1:n(2)
    center(:,i) = [centerOfMass(robot,joint_angles(:,i));1]
    Hft_ = Hft(kinematics3D(joint_angles(:,i),param));
    center(:,i) = Hft_*center(:,i);
    %rotate to plot based coordinate system.
    [x,y,z] = sphere;
    x = x*0.02;
    y = y*0.02;
    z = z*0.02;
    surf(x+center(3,i),y+center(2,i),z-center(1,i));
    hold on
    X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
    Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
    Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];
    X = 0.115*2*(X-0.5);
    Y = 0.065*2*(Y-0.5);
    Z = 1*(Z); 
    C='cyan'; 
    fill3(X,Y,Z,C,'FaceAlpha',0.5);
    X = [-1 1 1 -1];
    Y = [1 1 -1 -1];
    Z = [0 0 0 0];
    fill3(X,Y,Z,[211,211,211]/1000);
    hold off
    axis([-0.5 0.5 -0.5 0.5 0 1]);
    drawnow
    waitfor(r);
end
end

