function output = kinematics3D(q_in,param)
%% Map urdf joint angles to kinematics
q = [q_in(12),q_in(13),q_in(14),q_in(15),q_in(16),q_in(17),q_in(1),q_in(2),q_in(3),q_in(4),q_in(5),q_in(6)];

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

com_hip_roll = hip_roll/2;
com_hip_pitch = hip_pitch/2;
com_upper_leg = upper_leg/2;
com_lower_leg = lower_leg/2;
com_ankle = 0;
com_foot = 0;
com_upper_body = 0.2;

m_yaw = 0.067;
m_roll = 0.306;
m_u_leg = 0.387;
m_l_leg = 0.177;
m_ankle = 0.306;
m_foot = 0.205;
m_upper_body = 0.135 + 0.419 + 2.539 + (0.02 + 0.3 + 0.26)*2;

total_mass = m_yaw*2 + m_roll*2 + m_u_leg*2 + m_l_leg*2 + m_ankle*2 + m_foot*2 + m_upper_body;

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
%% Centre of Mass
% right leg com
CR1 = @(q1) A01*(rotz(q1)*roty(pi/2)*tranx(com_hip_roll));
CR2 = @(q1,q2) A01*A12(q1)*(rotz(q2)*rotx(-pi/2)*tranx(com_hip_pitch));
CR3 = @(q1,q2,q3) A01*A12(q1)*A23(q2)*(rotz(q3)*tranx(com_upper_leg));
CR4 = @(q1,q2,q3,q4) A01*A12(q1)*A23(q2)*A34(q3)*(rotz(q4)*tranx(com_lower_leg));
CR5 = @(q1,q2,q3,q4,q5) A01*A12(q1)*A23(q2)*A34(q3)*A45(q4)*(rotz(q5)*rotx(pi/2)*tranx(com_ankle));
CR6 = @(q1,q2,q3,q4,q5,q6) A01*A12(q1)*A23(q2)*A34(q3)*A45(q4)*A56(q5)*(rotz(q6));
%left leg com
CL1 = @(q7) A07*(rotz(q7)*roty(pi/2)*tranx(com_hip_roll));
CL2 = @(q7,q8) A07*A78(q7)*(rotz(q8)*rotx(-pi/2)*tranx(com_hip_pitch));
CL3 = @(q7,q8,q9) A07*A78(q7)*A89(q8)*(rotz(q9)*tranx(com_upper_leg));
CL4 = @(q7,q8,q9,q10) A07*A78(q7)*A89(q8)*A910(q9)*(rotz(q10)*tranx(com_lower_leg));
CL5 = @(q7,q8,q9,q10,q11) A07*A78(q7)*A89(q8)*A910(q9)*A1011(q10)*(rotz(q11)*rotx(pi/2)*tranx(com_ankle));
CL6 = @(q7,q8,q9,q10,q11,q12) A07*A78(q7)*A89(q8)*A910(q9)*A1011(q10)*A1112(q11)*(rotz(q12));
%torso and upper body com
CT = tranz(com_upper_body);


x = @(T) T(1,4);
y = @(T) T(2,4);
z = @(T) T(3,4);

com_x = (x(CR1(q(1)))*m_yaw + x(CR2(q(1),q(2)))*m_roll + x(CR3(q(1),q(2),q(3)))*m_u_leg + x(CR4(q(1),q(2),q(3),q(4)))*m_l_leg + x(CR5(q(1),q(2),q(3),q(4),q(5)))*m_ankle + x(CR6(q(1),q(2),q(3),q(4),q(5),q(6)))*m_foot + ...
    x(CL1(q(7)))*m_yaw + x(CL2(q(7),q(8)))*m_roll + x(CL3(q(7),q(8),q(9)))*m_u_leg + x(CL4(q(7),q(8),q(9),q(10)))*m_l_leg + x(CL5(q(7),q(8),q(9),q(10),q(11)))*m_ankle + x(CL6(q(7),q(8),q(9),q(10),q(11),q(12)))*m_foot+ ...
    x(CT)*m_upper_body)/total_mass;
com_y = (y(CR1(q(1)))*m_yaw + y(CR2(q(1),q(2)))*m_roll + y(CR3(q(1),q(2),q(3)))*m_u_leg + y(CR4(q(1),q(2),q(3),q(4)))*m_l_leg + y(CR5(q(1),q(2),q(3),q(4),q(5)))*m_ankle + y(CR6(q(1),q(2),q(3),q(4),q(5),q(6)))*m_foot + ...
    y(CL1(q(7)))*m_yaw + y(CL2(q(7),q(8)))*m_roll + y(CL3(q(7),q(8),q(9)))*m_u_leg + y(CL4(q(7),q(8),q(9),q(10)))*m_l_leg + y(CL5(q(7),q(8),q(9),q(10),q(11)))*m_ankle + y(CL6(q(7),q(8),q(9),q(10),q(11),q(12)))*m_foot+ ...
    y(CT)*m_upper_body)/total_mass;
com_z = (z(CR1(q(1)))*m_yaw + z(CR2(q(1),q(2)))*m_roll + z(CR3(q(1),q(2),q(3)))*m_u_leg + z(CR4(q(1),q(2),q(3),q(4)))*m_l_leg + z(CR5(q(1),q(2),q(3),q(4),q(5)))*m_ankle + z(CR6(q(1),q(2),q(3),q(4),q(5),q(6)))*m_foot + ...
    z(CL1(q(7)))*m_yaw + z(CL2(q(7),q(8)))*m_roll + z(CL3(q(7),q(8),q(9)))*m_u_leg + z(CL4(q(7),q(8),q(9),q(10)))*m_l_leg + z(CL5(q(7),q(8),q(9),q(10),q(11)))*m_ankle + z(CL6(q(7),q(8),q(9),q(10),q(11),q(12)))*m_foot+ ...
    z(CT)*m_upper_body)/total_mass;

output.com = [com_x;com_y;com_z];

%% Outputs        
output.T0R = A01*A12(q(1))*A23(q(2))*A34(q(3))*A45(q(4))*A56(q(5))*A6R(q(6));
output.T0L = A07*A78(q(7))*A89(q(8))*A910(q(9))*A1011(q(10))*A1112(q(11))*A12L(q(12));
if(param.supportFoot == "left_foot")
    output.T = output.T0L\output.T0R;
    output.xe = output.T(1:3,4);
    output.R = output.T(1:3,1:3);
    output.Htf = output.T0L;
else
    output.T = output.T0R\output.T0L;
    output.xe = output.T(1:3,4);
    output.R = output.T(1:3,1:3);
    output.Htf = output.T0R;
end

%transform to ankles
A06 = A01*A12(q(1))*A23(q(2))*A34(q(3))*A45(q(4))*A56(q(5));
A012 = A07*A78(q(7))*A89(q(8))*A910(q(9))*A1011(q(10))*A1112(q(11))*A12L(q(12));
output.RR = [0 0 -1;0 1 0;1 0 0]*A06(1:3,1:3);
output.RL = [0 0 -1;0 1 0;1 0 0]*A012(1:3,1:3);
end
