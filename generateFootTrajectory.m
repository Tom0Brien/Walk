function output = generateFootTrajectory(param,isHalfStep)
%% Helper Functions
position = @(transform) transform(1:3,4);
%% Half step case
if isHalfStep == true
   stepLength = param.stepLength/2;
else
   stepLength = param.stepLength;
end
%% Initial Position
init = position(getTransform(param.robot,param.initialConditions,param.swingFoot,param.supportFoot));
%% Generate spline trajectoy for swing foot to follow - X
D = diag(1:3,-1);
X0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
X1 = [stepLength 0];
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
%% Generate spline trajectoy for swing foot to follow - Z
D = diag(1:3,-1);
%initial position
Z0 = [0 0];
t0 = 0;
tt0 = t0.^(0:3).';
T0 = [tt0,D*tt0];
%apex of swing foot trajectory z = 0.15, velocity = 0;
Z1 = [param.stepHeight 0];
t1 = param.stepTime/4;
tt1 = t1.^(0:3).';
T1 = [tt1,D*tt1];
%back down to ground
Z2 = [0 0];
t2 = param.stepTime;
tt2 = t2.^(0:3).';
T2 = [tt2,D*tt2];
C1 = [Z0 Z1]/[T0 T1];
C2 = [Z1 Z2]/[T1 T2];
t = linspace(t0,t1,param.numSamples/2);
tt = t.^((0:3).');
z = C1*tt;
v = C1*D*tt;
t = linspace(t1,t2,param.numSamples/2);
tt = t.^((0:3).');
z = [z C2*tt];
v = [v C2*D*tt];
output = [x;zeros(1,param.numSamples);z];
% Shift initial point to swing foot
output = output + position(getTransform(param.robot,param.initialConditions,param.swingFoot,param.supportFoot));
end

