function output = generateCoMTrajectory(param)
position = @(transform) transform(1:3,4);
%% Get CoM position in support foot space
Htf = getTransform(param.robot,param.initialConditions,'base',param.supportFoot);
rCTt = centerOfMass(param.robot,param.initialConditions);
rCFf = Htf*[rCTt;1]
%% Generate spline trajectoy for swing foot to follow - X
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
figure(3);
% plot(t,x,t,v);
legend('x position','x velocity');
output = [x;zeros(1,param.numSamples);zeros(1,param.numSamples)];

%% Plot trajectory in swing and support foot space
figure(4);
n = param.numSamples;
plot3(x,rCFf(2)*ones(1,n),rCFf(3)*ones(1,n));
figure(5);
Hft = getTransform(param.robot,param.initialConditions,param.supportFoot,'base');
for i=1:n
    rCFt(:,i) = Hft*[x(i);rCFf(2);rCFf(3);1];
end
show(param.robot,param.initialConditions);
hold on;
plot3(rCFt(1,:),rCFt(2,:),rCFt(3,:));
grid on;
end

