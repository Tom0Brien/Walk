function output = generateCoMTrajectory(param)
position = @(transform) transform(1:3,4);
%% Get CoM foot position
Hft = getTransform(param.robot,param.initialConditions,'torso',param.supportFoot);
CoMFootPos = Hft*[centerOfMass(param.robot,param.initialConditions);1]
%% Generate spline trajectoy for swing foot to follow - X
D = diag(1:3,-1);
X0 = [CoMFootPos(1)  0];
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
Y0 = [CoMFootPos(2) 0];
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
output = [y];
% Shift to swing foot
end

