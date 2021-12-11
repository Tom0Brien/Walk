FK = Kinematics();
%% Forward kinematics
robot = importrobot('NUgus.urdf');
% smimport('NUgus.urdf')
robot.DataFormat = 'column';
%% Params
p = gaitParameters(robot);
p.support_foot = 'left_foot';
p.swingFoot = 'right_foot';
q=zeros(20,1);

%% fk
% for i=1:100000
% %     x = FK.xe(q,p)
%     getTransform(robot,q,'left_foot')
% end

%% ik
% joints0 = zeros(20,1);
% cost = @(q) costFunctionStatic(q,[0;0;0],joints0,p,1);
% nlconstraint = @(q) nonlconFoot(q,p,joints0);
% options = optimset('algorithm','interior-point');
%     
% for i=1:100
%     joints_opt = fmincon(cost,joints0,[],[],[],[],[],[],nlconstraint, options);
% end

%% ik
ik = inverseKinematics('RigidBodyTree',robot);
tform = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
for i=1:100
    ik('left_foot',tform,[0.25 0.25 0.25 1 1 1],zeros(20,1))
end

