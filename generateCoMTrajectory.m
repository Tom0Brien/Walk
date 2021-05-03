function output = generateCoMTrajectory(param)
position = @(transform) transform(1:3,4);
%% Get CoM position in support foot space
Htf = getTransform(param.robot,param.initialConditions,'torso',param.supportFoot);
rCTt = centerOfMass(param.robot,param.initialConditions);
rCFf = Htf*[rCTt;1];
%% Generate spline trajectoy for CoM to follow
D = diag(1:3,-1);
X0 = [rCFf(1) 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
X1 = [0 0];
t1 = param.stepTime;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
C = [X0 X1]/[T0 T1];
t = linspace(t0,t1,param.numSamples);
tt = t.^((0:3).');
x = C*tt;
v = C*D*tt;
figure(1);
plot(t,x,t,v);
legend('x position','x velocity');
%% Generate spline trajectoy for swing foot to follow - Y
D = diag(1:3,-1);
%initial position
Y0 = [rCFf(2) 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
Y1 = [0 0];
t1 = param.stepTime;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
C = [Y0 Y1]/[T0 T1];
t = linspace(t0,t1,param.numSamples);
tt = t.^((0:3).');
y = C*tt;
v = C*D*tt;

%% Plot 
figure(4);
n = param.numSamples;
plot3(x,rCFf(2)*ones(1,n),rCFf(3)*ones(1,n));
figure(5);
Hft = getTransform(param.robot,param.initialConditions,param.supportFoot,'torso');
for i=1:n
    rCFt(:,i) = Hft*[x(i);y(i);rCFf(3);1];
end
show(param.robot,param.initialConditions);
hold on;
plot3(rCTt(1),rCTt(2),rCTt(3),'o')
hold on;
plot3(rCFt(1,:),rCFt(2,:),rCFt(3,:));
grid on;
%% Output torso space CoM trajectory
output = [rCFt(1,:);rCFt(2,:);rCFt(3,:)]
end

