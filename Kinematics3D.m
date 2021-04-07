function output = Kinematics3D(q)
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
Hb1 = @(q12) [Rz(q12)*eulerRotation([0,0,pi/2]), [0;-0.055;0]
              0 0 0 1;
];
%right_hip_roll
H12 = @(q13) [Ry(-q13), [0;0;-0.06]
              0 0 0 1;
            ];
%right_hip_pitch
H23 = @(q14) [Rx(q14)*eulerRotation([-0.05,0,0]), [0;-0.01;0.01]
              0 0 0 1;
            ];
%right_knee_pitch
H34 = @(q15) [Rx(q15)*eulerRotation([0.05,0,0]), [0; 0.00999; -0.19975]
              0 0 0 1;
            ];
%right_ankle_pitch
H45 = @(q16) [Rx(-q16)*eulerRotation([pi,0,-pi]), [0;    0.01; -0.2]
              0 0 0 1;
            ];
%right_ankle_roll
H56 = @(q17) [Ry(-q17)*eulerRotation([-pi,0,-pi]), [0;    0.04;  0]
              0 0 0 1;
            ];
%left_hip_yaw
Hb6 = @(q1) [Rz(q1)*eulerRotation([0,0,pi/2]), [0;0.055;0]
              0 0 0 1;
            ]; 
%left_hip_roll
H67 = @(q2) [Ry(-q2), [0;0;-0.06]
              0 0 0 1;
            ];
%left_hip_pitch
H78 = @(q3) [Rx(q3)*eulerRotation([-0.05,0,0]), [0;-0.01;0]
              0 0 0 1;
            ];
%left_knee_pitch
H89 = @(q4) [Rx(q4)*eulerRotation([0.05,0,0]), [0;0.00949;-0.18976]
              0 0 0 1;
            ];
%left_ankle_pitch
H910 = @(q5) [Rx(-q5)*eulerRotation([pi,0,-pi]), [0;    0.01; -0.2]
              0 0 0 1;
            ];
%left_ankle_roll
H1011 = @(q6) [Ry(-q6)*eulerRotation([-pi,0,-pi]), [0;    0.04;  0]
              0 0 0 1;
            ];
%% Homogeneous transforms support foot based
%torso to left foot
Hb10 = @(q1,q2,q3,q4,q5,q6) Hb6(q1)*H67(q2)*H78(q3)*H89(q4)*H910(q5)*H1011(q6);
%left foot to torso
H10b = @(q1,q2,q3,q4,q5,q6) inv(Hb10(q1,q2,q3,q4,q5,q6));
%torso to right foot
Hb5 =  @(q12,q13,q14,q15,q16,q17) Hb1(q12)*H12(q13)*H23(q14)*H34(q15)*H45(q16)*H56(q17);
%left foot to right foot
H105 = @(q1,q2,q3,q4,q5,q6,q12,q13,q14,q15,q16,q17) H10b(q1,q2,q3,q4,q5,q6)*Hb5(q12,q13,q14,q15,q16,q17);
%right foot to left foot
H510 = @(q1,q2,q3,q4,q5,q6,q12,q13,q14,q15,q16,q17) inv(H105(q1,q2,q3,q4,q5,q6,q12,q13,q14,q15,q16,q17));

% H105(0,0,0,0,0,0,0,0,0,0)
     
%% Outputs        
% output transform from torso to right foot
output.torsoToRightFoot = Hb1(q(12))*H12(q(13))*H23(q(14))*H34(q(15))*H45(q(16))*H56(q(17));
output.torsoToLeftFoot = Hb6(q(1))*H67(q(2))*H78(q(3))*H89(q(4))*H910(q(5))*H1011(q(6));
output.leftFootToTorso = inv(output.torsoToLeftFoot);
output.leftFootToTorsoPosition = output.leftFootToTorso(1:3,4);
output.leftPosition = output.torsoToLeftFoot(1:3,4);
output.rightPosition = output.torsoToRightFoot(1:3,4);
output.leftToRightFoot = H105(q(1),q(2),q(3),q(4),q(5),q(6),q(12),q(13),q(14),q(15),q(16),q(17));
output.leftToRightFootPosition = output.leftToRightFoot(1:3,4);
output.rightToLeftFoot = H510(q(1),q(2),q(3),q(4),q(5),q(6),q(12),q(13),q(14),q(15),q(16),q(17));
output.rightToLeftFootPosition = output.rightToLeftFoot(1:3,4);
end
