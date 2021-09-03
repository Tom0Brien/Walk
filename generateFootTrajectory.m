function output = generateFootTrajectory(p)
if(p.step_count == 1)
    step_length = p.step_length/2;
else
    step_length = p.step_length;
end
%% Generate spline trajectoy for swing foot to follow - X
D = diag(1:3,-1);
X0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
X1 = [step_length 0];
t1 = p.step_time;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
C = [X0 X1]/[T0 T1];
t = linspace(t0,t1,p.N);
tt = t.^((0:3).');
x = C*tt;
v = C*D*tt;
%% Generate spline trajectoy for swing foot to follow - Z
D = diag(1:3,-1);
%initial position
Z0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
%apex of swing foot trajectory z = 0.15, velocity = 0;
Z1 = [p.step_height 0];
t1 = p.step_time/4;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
%back down to ground
Z2 = [0 0];
t2 = p.step_time;
tt2 = t2.^(0:3).';
T2 = [tt2,D*tt2];
C1 = [Z0 Z1]/[T0 T1];
C2 = [Z1 Z2]/[T1 T2];
t = linspace(t0,t1,p.N/2);
tt = t.^((0:3).');
z = C1*tt;
v = C1*D*tt;
t = linspace(t1,t2,p.N/2);
tt = t.^((0:3).');
z = [z C2*tt];
v = [v C2*D*tt];
output = [-z;zeros(1,p.N);x];
% Shift initial point to swing foot
xe = @(kinematics) kinematics.xe;
if p.support_foot == "left_foot"
    y = -p.step_width;
else
    y =  p.step_width;
end
FK = Kinematics();
output = output + FK.xe(p.initial_conditions,p); % [1 0 0;0 1 0;0 0 1]*xe(kinematics3D(p.initial_conditions,p));% [0;y;0];
%% Plot trajectory in torso space
show(p.robot,p.initial_conditions);
hold on;
Htf = @(kinematics) kinematics.Htf;
trajectory = zeros(4,p.N);
for i=1:p.N
    trajectory(:,i) = Htf(kinematics3D(p.initial_conditions,p))*[output(:,i);1];
end
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:));
end

