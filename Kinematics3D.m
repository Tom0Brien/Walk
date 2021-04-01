function output = Kinematics3D(q,foot)
%% Axis rotations
Rx = @(phi) [1,0,0;...
             0,cos(phi),-sin(phi);...
             0,sin(phi),cos(phi);...
            ];
Ry = @(theta)[cos(theta), 0,sin(theta); ...
              0,1,0; ...
              -sin(theta), 0, cos(theta);...
              ];

Rz = @(psi)[cos(psi) -sin(psi) 0;...
            sin(psi) cos(psi) 0;...
            0 0 1;...
           ];

%% Homogeneous Transforms Trunk Based Coordinates
%right_hip_yaw
Hb1 = @(q12) [eulerRotation([0,0,pi/2])*Rz(q12), [0;-0.055;0]
              0 0 0 1;
];
%right_hip_roll
H12 = @(q13) [Ry(-q13), [0;0;-0.06]
              0 0 0 1;
            ];
%right_hip_pitch
H23 = @(q14) [eulerRotation([-0.05,0,0])*Rx(q14), [0;-0.01;0.01]
              0 0 0 1;
            ];
%right_knee_pitch
H34 = @(q15) [eulerRotation([0.05,0,0])*Rx(q15), [0; 0.00999; -0.19975]
              0 0 0 1;
            ];
%right_ankle_pitch
H45 = @(q16) [eulerRotation([pi,0,-pi])*Rx(-q16), [0;    0.01; -0.2]
              0 0 0 1;
            ];
%right_ankle_roll
H54 = @(q17) [eulerRotation([-pi,0,-pi])*Ry(-q17), [0;    0.04;  0]
              0 0 0 1;
            ];
%left_hip_yaw
Hb6 = @(q1) [eulerRotation([0,0,pi/2])*Rz(q1), [0;0.055;0]
              0 0 0 1;
            ]; 
%left_hip_roll
H67 = @(q2) [Ry(-q2), [0;0;-0.06]
              0 0 0 1;
            ];
%left_hip_pitch
H78 = @(q3) [eulerRotation([-0.05,0,0])*Rx(q3), [0;-0.01;0]
              0 0 0 1;
            ];
%left_knee_pitch
H89 = @(q4) [eulerRotation([0.05,0,0])*Rx(q4), [0;0.00949;-0.18976]
              0 0 0 1;
            ];
%left_ankle_pitch
H910 = @(q5) [eulerRotation([pi,0,-pi])*Rx(-q5), [0;    0.01; -0.2]
              0 0 0 1;
            ];
%left_ankle_roll
H1011 = @(q6) [eulerRotation([-pi,0,-pi])*Ry(-q6), [0;    0.04;  0]
              0 0 0 1;
            ];
%% Homogenous transforms support foot based
%torso to left foot
Hb10 = @(q1,q2,q3,q4,q5) Hb6(q1)*H67(q2)*H78(q3)*H89(q4)*H910(q5);
%torso to right foot
Hb5 =  @(q12,q13,q14,q15,q16) Hb1(q12)*H12(q13)*H23(q14)*H34(q15)*H45(q16)
%left foot to torso
H10b = @(q1,q2,q3,q4,q5) inv(Hb10(q1,q2,q3,q4,q5));
%left foot to right foot
H105 = @(q1,q2,q3,q4,q5,q12,q13,q14,q15,q16) H10b(q1,q2,q3,q4,q5)*Hb5(q12,q13,q14,q15,q16);

H510 = @(q1,q2,q3,q4,q5,q12,q13,q14,q15,q16) inv(H105(q1,q2,q3,q4,q5,q12,q13,q14,q15,q16));


     
%% Outputs        
% output transform from torso to right foot
output.rightTransform = Hb1(q(12))*H12(q(13))*H23(q(14))*H34(q(15))*H45(q(16));
output.leftTransform = Hb6(q(1))*H67(q(2))*H78(q(3))*H89(q(4))*H910(q(5));
output.leftPosition = output.leftTransform(1:3,4);
output.leftToRightFoot = H105(q(1),q(2),q(3),q(4),q(5),q(12),q(13),q(14),q(15),q(16));
output.RightToLeftFoot = H510(q(1),q(2),q(3),q(4),q(5),q(12),q(13),q(14),q(15),q(16));
end
