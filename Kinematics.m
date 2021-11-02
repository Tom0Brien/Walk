classdef Kinematics
   properties
      p
   end
   methods (Static)
       function [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = FK(q_in, p)
            %% Maps joint angles to forward kinematic model            
            q = [q_in(12),q_in(13),q_in(14),q_in(15),q_in(16),q_in(17),q_in(1),q_in(2),q_in(3),q_in(4),q_in(5),q_in(6)];
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
            A6R = rotz(q(6))*tranx(p.foot_z_offset);
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
            A12L =  rotz(q(12))*tranx(p.foot_z_offset);
       end
       function [Hsp_] = Hsp(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T0R = A01*A12*A23*A34*A45*A56*A6R;
           T0L = A07*A78*A89*A910*A1011*A1112*A12L;
           if(param.support_foot == "left_foot")
                Hsp_ = T0L\T0R;
            else
                Hsp_ = T0R\T0L;
            end
       end
       function [xe_] = xe(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T0R = A01*A12*A23*A34*A45*A56*A6R;
           T0L = A07*A78*A89*A910*A1011*A1112*A12L;
           if(param.support_foot == "left_foot")
                Hsp_ = T0L\T0R;
                xe_ = Hsp_(1:3,4);
            else
                Hsp_ = T0R\T0L;
                xe_ = Hsp_(1:3,4);
            end
       end
       function [R_] = R(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T0R = A01*A12*A23*A34*A45*A56*A6R;
           T0L = A07*A78*A89*A910*A1011*A1112*A12L;
           if(param.support_foot == "left_foot")
                Hsp_ = T0L\T0R;
                R_ = Hsp_(1:3,1:3);
            else
                Hsp_ = T0R\T0L;
                R_ = Hsp_(1:3,1:3);
            end
       end
       function [Htp_] = Htp(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T0R = A01*A12*A23*A34*A45*A56*A6R;
           T0L = A07*A78*A89*A910*A1011*A1112*A12L;
           if(param.support_foot == "left_foot")
                Htp_ = T0L;
           else
                Htp_ = T0R;
            end
       end
       function [xs_] = xs(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T0R = A01*A12*A23*A34*A45*A56*A6R;
           T0L = A07*A78*A89*A910*A1011*A1112*A12L;
           if(param.support_foot == "left_foot")
                xs_ = T0L(1:3,4);
           else
                xs_ = T0R(1:3,4);
           end
       end
       function [RR_] = RR(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T06 = A01*A12*A23*A34*A45*A56;
           RR_ = T06(1:3,1:3);
       end
       function [RL_] = RL(q,param)
           [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q,param);
           T012 = A07*A78*A89*A910*A1011*A1112;
           RL_ = T012(1:3,1:3);
       end
       function [com_] = CoM(q_in,param)
            %% Params
            [A01,A12,A23,A34,A45,A56,A6R,A07,A78,A89,A910,A1011,A1112,A12L] = Kinematics.FK(q_in,param);
            q = [q_in(12),q_in(13),q_in(14),q_in(15),q_in(16),q_in(17),q_in(1),q_in(2),q_in(3),q_in(4),q_in(5),q_in(6)];

            com_hip_yaw = 0.0461;
            com_hip_roll = 0.015730;
            com_upper_leg = 0.086820;
            com_lower_leg = 0.1;
            com_ankle = 0.015730;
            com_foot = 0.030800;
            com_upper_body = 0.102150;

            m_yaw = 0.067;
            m_roll = 0.306;
            m_u_leg = 0.387;
            m_l_leg = 0.177;
            m_ankle = 0.306;
            m_foot = 0.205;
            m_upper_body = 0.135 + 0.419 + 2.539 + (0.02 + 0.3 + 0.26)*2;

            total_mass = m_yaw*2 + m_roll*2 + m_u_leg*2 + m_l_leg*2 + m_ankle*2 + m_foot*2 + m_upper_body;
            %% Centre of Mass
            % right leg com
            CR1 = A01*(rotz(q(1))*roty(pi/2)*tranx(com_hip_yaw));
            CR2 = A01*A12*(rotz(q(2))*rotx(-pi/2)*tranx(com_hip_roll));
            CR3 = A01*A12*A23*(rotz(q(3))*tranx(com_upper_leg));
            CR4 = A01*A12*A23*A34*(rotz(q(4))*tranx(com_lower_leg));
            CR5 = A01*A12*A23*A34*A45*(rotz(q(5))*rotx(pi/2)*tranx(com_ankle));
            CR6 = A01*A12*A23*A34*A45*A56*(rotz(q(6))*tranx(com_foot));
            %left leg com
            CL1 = A07*(rotz(q(7))*roty(pi/2)*tranx(com_hip_yaw));
            CL2 = A07*A78*(rotz(q(8))*rotx(-pi/2)*tranx(com_hip_roll));
            CL3 = A07*A78*A89*(rotz(q(9))*tranx(com_upper_leg));
            CL4 = A07*A78*A89*A910*(rotz(q(10))*tranx(com_lower_leg));
            CL5 = A07*A78*A89*A910*A1011*(rotz(q(11))*rotx(pi/2)*tranx(com_ankle));
            CL6 = A07*A78*A89*A910*A1011*A1112*(rotz(q(12))*tranx(com_foot));
            %torso and upper body com
            CT = tranz(com_upper_body);


            x = @(T) T(1,4);
            y = @(T) T(2,4);
            z = @(T) T(3,4);
            
            mass = [m_yaw, m_roll, m_u_leg, m_l_leg, m_ankle, m_foot, m_yaw, m_roll, m_u_leg, m_l_leg, m_ankle, m_foot, m_upper_body];
            transforms = cat(3,CR1,CR2,CR3,CR4,CR5,CR6,CL1,CL2,CL3,CL4,CL5,CL6,CT);
            com_ = [0;0;0];

            for i = 1:size(mass,2)
                com_ = [com_(1) + transforms(1,4,i)*mass(i); ...
                        com_(2) + transforms(2,4,i)*mass(i);
                        com_(3) + transforms(3,4,i)*mass(i);];
            end
            com_ = com_/total_mass;
%             

%             com_x = (x(CR1)*m_yaw + x(CR2)*m_roll + x(CR3)*m_u_leg + x(CR4)*m_l_leg + x(CR5)*m_ankle + x(CR6)*m_foot + ...
%                      x(CL1)*m_yaw + x(CL2)*m_roll + x(CL3)*m_u_leg + x(CL4)*m_l_leg + x(CL5)*m_ankle + x(CL6)*m_foot+ ...
%                      x(CT)*m_upper_body)/total_mass;
%             com_y = (y(CR1)*m_yaw + y(CR2)*m_roll + y(CR3)*m_u_leg + y(CR4)*m_l_leg + y(CR5)*m_ankle + y(CR6)*m_foot + ...
%                      y(CL1)*m_yaw + y(CL2)*m_roll + y(CL3)*m_u_leg + y(CL4)*m_l_leg + y(CL5)*m_ankle + y(CL6)*m_foot+ ...
%                      y(CT)*m_upper_body)/total_mass;
%             com_z = (z(CR1)*m_yaw + z(CR2)*m_roll + z(CR3)*m_u_leg + z(CR4)*m_l_leg + z(CR5)*m_ankle + z(CR6)*m_foot + ...
%                      z(CL1)*m_yaw + z(CL2)*m_roll + z(CL3)*m_u_leg + z(CL4)*m_l_leg + z(CL5)*m_ankle + z(CL6)*m_foot+ ...
%                      z(CT)*m_upper_body)/total_mass;
%             com_ = [com_x;com_y;com_z];
                
       end  
   end
end

