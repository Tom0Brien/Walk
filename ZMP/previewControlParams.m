function [A_d, B_d, C_d, Gi, Gx, Gd] = previewControlParams(zc, dt, t_preview, Qe, R)

g = 9.81; % gravity

% Matrix A, B, C, and D from cart table model
A = [0 1 0;
     0 0 1;
     0 0 0];
B = [0;
     0;
     1];
C = [1 0 -zc/g];
D = 0;

% Convert continuous system to discrete system
sys_c = ss(A, B, C, D);            
sys_d = c2d(sys_c, dt);
[A_d, B_d, C_d, D_d] = ssdata(sys_d);

% A, B, C matrix for LQI control
A_tilde = [1 C_d*A_d;
           zeros(3,1) A_d];
B_tilde = [C_d*B_d;
           B_d];
C_tilde = [1 0 0 0];

% Q matrix
Q = [Qe 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

% Find optimal gain and ricatti equation with dlqr function
% K = optimal gain
% P = ricatti equation
[K, P] = dlqr(A_tilde, B_tilde, Q, R);

Gi = K(1);
Gx = K(2:end);

% Calculating preview gain
N = 0:dt:t_preview;
Gd = zeros(1, length(N));
Gd(1,1) = -Gi;

Ac_tilde = A_tilde - B_tilde * K;

I_tilde = [1;0;0;0];
X_tilde = -Ac_tilde'*P*I_tilde;

for i = 2:length(N)
    Gd(1,i) = (R+B_tilde'*P*B_tilde)^(-1)*B_tilde'*X_tilde;
    X_tilde = Ac_tilde'*X_tilde;
end
end

