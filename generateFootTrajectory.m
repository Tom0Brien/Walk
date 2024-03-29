function rSPp = generateFootTrajectory(p)
if(p.step_count == 1)
    step_length_x = p.step_length_x/2;
else
    step_length_x = p.step_length_x;
end
%% Generate spline trajectoy for swing foot to follow - X
D = diag(1:3,-1);
X0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
X1 = [step_length_x 0];
t1 = p.step_time;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
C = [X0 X1]/[T0 T1];
t = linspace(t0,t1,p.N);
tt = t.^((0:3).');
x = C*tt;
%% Generate spline trajectoy for swing foot to follow - Z
D = diag(1:3,-1);
%initial position
Z0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
%apex of swing foot trajectory z = 0.15, velocity = 0;
Z1 = [p.step_height 0];
t1 = p.step_time/5;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
%back down to ground
Z2 = [0 0];
t2 = p.step_time;
tt2 = t2.^(0:3).';
T2 = [tt2,D*tt2];
C1 = [Z0 Z1]/[T0 T1];
C2 = [Z1 Z2]/[T1 T2];
t = linspace(t0,t1,floor(p.N/2));
tt = t.^((0:3).');
z = C1*tt;
t = linspace(t1,t2,p.N-floor(p.N/2));
tt = t.^((0:3).');
z = [z C2*tt];
%% Generate spline trajectoy for swing foot to follow - Y
D = diag(1:3,-1);
Y0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
Y1 = [p.step_length_y 0];
t1 = p.step_time;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
C = [Y0 Y1]/[T0 T1];
t = linspace(t0,t1,p.N);
tt = t.^((0:3).');
y = C*tt;
%% Pack splines together
rSPp = [-z;y;x];
FK = Kinematics();
rSPp = rSPp + [1,0,0;0,0,0;0,0,1]*FK.xe(p.initial_conditions,p) +[0;(-1)^(p.step_count)*p.step_width;0];
%% Plot trajectory in torso space
figure
show(p.robot,p.initial_conditions);
hold on;
trajectory = zeros(4,p.N);
for i=1:p.N
    trajectory(:,i) = FK.Htp(p.initial_conditions,p)*[rSPp(:,i);1];
end
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:));
end

