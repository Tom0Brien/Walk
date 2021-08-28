function output = Kinematics3D(q_in,param)
%% Map urdf joint angles to kinematics
q = [q_in(12),q_in(13),q_in(14),q_in(15),q_in(16),q_in(17),q_in(1),q_in(2),q_in(3),q_in(4),q_in(5),q_in(6)];
%% Carnogical stuff
rotx = @(phi) [1,0,0,0;...
             0,cos(phi),-sin(phi),0;...
             0,sin(phi),cos(phi),0;...
             0,0,0,1;
            ];
roty = @(theta)[cos(theta), 0,sin(theta),0; ...
              0,1,0,0; ...
              -sin(theta), 0, cos(theta),0;...
              0,0,0,1
              ];

rotz = @(psi)[cos(psi) -sin(psi) 0 0;...
            sin(psi) cos(psi) 0 0;...
            0 0 1 0;...
            0 0 0 1
           ];
       
tranx = @(x) [[eye(3);zeros(1,3)],[x;0;0;1]];
trany = @(y) [[eye(3);zeros(1,3)],[0;y;0;1]];
tranz = @(z) [[eye(3);zeros(1,3)],[0;0;z;1]];
%% Params
hip_yaw = 0.055;
hip_roll = 0.06;
hip_pitch = 0;
upper_leg = 0.2;
lower_leg = 0.2;
ankle = 0;
%% Homogeneous Transforms Trunk Based Coordinates
%right_hip_yaw
A01 = trany(-hip_yaw);
%right_hip_roll
A12 = @(q1) rotz(q1)*roty(pi/2)*tranx(hip_roll);
%right_hip_pitch
A23 = @(q2) rotz(q2)*rotx(-pi/2)*tranx(hip_pitch);
%right_upper_leg
A34 = @(q3) rotz(q3)*tranx(upper_leg);
%right_lower_leg
A45 = @(q4) rotz(q4)*tranx(lower_leg);
%right_ankle
A56 = @(q5) rotz(q5)*rotx(pi/2)*tranx(ankle);
%right_foot
A6R = @(q6) rotz(q6);
%right_hip_yaw
A07 = trany(hip_yaw);
%left_hip_roll
A78 = @(q7) rotz(q7)*roty(pi/2)*tranx(hip_roll);
%left_hip_pitch
A89 = @(q8) rotz(q8)*rotx(-pi/2)*tranx(hip_pitch);
%left_upper_leg
A910 = @(q9) rotz(q9)*tranx(upper_leg);
%left_lower_leg
A1011 = @(q10) rotz(q10)*tranx(lower_leg);
%left_ankle
A1112 = @(q11) rotz(q11)*rotx(pi/2)*tranx(ankle);
%left_foot
A12L = @(q12) rotz(q12);
     
%% Outputs        
T0R = A01*A12(q(1))*A23(q(2))*A34(q(3))*A45(q(4))*A56(q(5))*A6R(q(6))
T0L = A07*A78(q(7))*A89(q(8))*A910(q(9))*A1011(q(10))*A1112(q(11))*A12L(q(12))
param.support_foot = 'left_foot';
if(param.support_foot == 'left_foot')
    output.end_effector = inv(T0L)*T0R;
else
    output.end_effector = inv(T0R)*T0L;
end
end
