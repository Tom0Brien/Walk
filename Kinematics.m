classdef Kinematics
   properties
      p
   end
   methods (Static)
       function [Hsp_, xe_, R_, Htf_, xs_,RR_,RL_] = FK(q_in, param)
            %% Maps joint angles to forward kinematic model            
            q = [q_in(12),q_in(13),q_in(14),q_in(15),q_in(16),q_in(17),q_in(1),q_in(2),q_in(3),q_in(4),q_in(5),q_in(6)];
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
            T0R = A01*A12*A23*A34*A45*A56*A6R;
            T0L = A07*A78*A89*A910*A1011*A1112*A12L;
            if(param.supportFoot == "left_foot")
                Hsp_ = T0L\T0R;
                xe_ = Hsp_(1:3,4);
                R_ = Hsp_(1:3,1:3);
                Htf_ = T0L;
                xs_ = T0L(1:3,4);
            else
                Hsp_ = T0R\T0L;
                xe_ = Hsp_(1:3,4);
                R_ = Hsp_(1:3,1:3);
                Htf_ = T0R;
                xs_ = T0R(1:3,4);
            end
            %Rotation to ankles
            A06 = A01*A12*A23*A34*A45*A56;
            A012 = A07*A78*A89*A910*A1011*A1112*A12L;
            RR_ = [0 0 -1;0 1 0;1 0 0]*A06(1:3,1:3);
            RL_ = [0 0 -1;0 1 0;1 0 0]*A012(1:3,1:3);
       end
       function [Hsp_] = Hsp(q,param)
           [Hsp_, ~, ~, ~, ~,~,~] = Kinematics.FK(q,param);
       end
       function [xe_] = xe(q,param)
           [~, xe_, ~, ~, ~,~,~] = Kinematics.FK(q,param);
       end
       function [R_] = R(q,param)
           [~, ~, R_, ~, ~,~,~] = Kinematics.FK(q,param);
       end
       function [Htf_] = Htf(q,param)
           [~, ~, ~, Htf_, ~,~,~] = Kinematics.FK(q,param);
       end
       function [xs_] = xs(q,param)
           [~, ~, ~, ~, xs_,~,~] = Kinematics.FK(q,param);
       end
       function [RR_] = RR(q,param)
           [~, ~, ~, ~, ~,RR_,~] = Kinematics.FK(q,param);
       end
       function [RL_] = RL(q,param)
           [~, ~, ~, ~, ~,~,RL_] = Kinematics.FK(q,param);
       end  
   end
end

