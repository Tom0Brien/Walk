function output = plotCoM(joint_angles,param)
%% Helper Functions
position = @(transform) transform(1:3,4);
%% Plot Robot Configurations
figure(1);
robot = param.robot;
framesPerSecond = 10;
r = rateControl(framesPerSecond);
%% Initialize video
n = size(joint_angles);
center = zeros(4,n(2));
foot = zeros(3,n(2));
for i = 1:n(2)
    center(:,i) = [centerOfMass(robot,joint_angles(:,i));1];
    Hft = getTransform(robot,joint_angles(:,i),'torso',param.supportFoot);
    center(:,i) = Hft*center(:,i);
    [x,y,z] = sphere;
    x = x*0.025;
    y = y*0.025;
    z = z*0.025;
    surf(x+center(1,i),y+center(2,i),z+center(3,i));
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

