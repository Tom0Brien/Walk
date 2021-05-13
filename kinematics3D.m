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
com_upper_body = 0.15;

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
A12 = rotz(q(1))*roty(pi/2)*tranx(hip_roll);
%right_hip_pitch
A23 = rotz(q(2))*rotx(-pi/2)*tranx(hip_pitch);
%right_upper_leg
A34 = rotz(q(3))*tranx(upper_leg);
%right_lower_leg
A45 = rotz(q(4))*tranx(lower_leg);
%right_ankle
A56 = rotz(q(5))*rotx(pi/2)*tranx(ankle);
%right_foot
A6R = rotz(q(6));
%right_hip_yaw
A07 = trany(hip_yaw);
%left_hip_roll
A78 = rotz(q(7))*roty(pi/2)*tranx(hip_roll);
%left_hip_pitch
A89 = rotz(q(8))*rotx(-pi/2)*tranx(hip_pitch);
%left_upper_leg
A910 = rotz(q(9))*tranx(upper_leg);
%left_lower_leg
A1011 =  rotz(q(10))*tranx(lower_leg);
%left_ankle
A1112 =  rotz(q(11))*rotx(pi/2)*tranx(ankle);
%left_foot
A12L =  rotz(q(12));
%% Centre of Mass
% right leg com
CR1 = A01*(rotz(q(1))*roty(pi/2)*tranx(com_hip_roll));
CR2 = A01*A12*(rotz(q(2))*rotx(-pi/2)*tranx(com_hip_pitch));
CR3 = A01*A12*A23*(rotz(q(3))*tranx(com_upper_leg));
CR4 = A01*A12*A23*A34*(rotz(q(4))*tranx(com_lower_leg));
CR5 = A01*A12*A23*A34*A45*(rotz(q(5))*rotx(pi/2)*tranx(com_ankle));
CR6 = A01*A12*A23*A34*A45*A56*(rotz(q(6)));
%left leg com
CL1 = A07*(rotz(q(7))*roty(pi/2)*tranx(com_hip_roll));
CL2 = A07*A78*(rotz(q(8))*rotx(-pi/2)*tranx(com_hip_pitch));
CL3 = A07*A78*A89*(rotz(q(9))*tranx(com_upper_leg));
CL4 = A07*A78*A89*A910*(rotz(q(10))*tranx(com_lower_leg));
CL5 = A07*A78*A89*A910*A1011*(rotz(q(11))*rotx(pi/2)*tranx(com_ankle));
CL6 = A07*A78*A89*A910*A1011*A1112*(rotz(q(12)));
%torso and upper body com
CT = tranz(com_upper_body);


x = @(T) T(1,4);
y = @(T) T(2,4);
z = @(T) T(3,4);

com_x = (x(CR1)*m_yaw + x(CR2)*m_roll + x(CR3)*m_u_leg + x(CR4)*m_l_leg + x(CR5)*m_ankle + x(CR6)*m_foot + ...
         x(CL1)*m_yaw + x(CL2)*m_roll + x(CL3)*m_u_leg + x(CL4)*m_l_leg + x(CL5)*m_ankle + x(CL6)*m_foot+ ...
         x(CT)*m_upper_body)/total_mass;
com_y = (y(CR1)*m_yaw + y(CR2)*m_roll + y(CR3)*m_u_leg + y(CR4)*m_l_leg + y(CR5)*m_ankle + y(CR6)*m_foot + ...
         y(CL1)*m_yaw + y(CL2)*m_roll + y(CL3)*m_u_leg + y(CL4)*m_l_leg + y(CL5)*m_ankle + y(CL6)*m_foot+ ...
         y(CT)*m_upper_body)/total_mass;
com_z = (z(CR1)*m_yaw + z(CR2)*m_roll + z(CR3)*m_u_leg + z(CR4)*m_l_leg + z(CR5)*m_ankle + z(CR6)*m_foot + ...
         z(CL1)*m_yaw + z(CL2)*m_roll + z(CL3)*m_u_leg + z(CL4)*m_l_leg + z(CL5)*m_ankle + z(CL6)*m_foot+ ...
         z(CT)*m_upper_body)/total_mass;

output.com = [com_x;com_y;com_z];

%% Outputs        
output.T0R = A01*A12*A23*A34*A45*A56*A6R;
output.T0L = A07*A78*A89*A910*A1011*A1112*A12L;
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
A06 = A01*A12*A23*A34*A45*A56;
A012 = A07*A78*A89*A910*A1011*A1112*A12L;
output.RR = [0 0 -1;0 1 0;1 0 0]*A06(1:3,1:3);
output.RL = [0 0 -1;0 1 0;1 0 0]*A012(1:3,1:3);
end
