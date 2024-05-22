function Lqr = lqr_control(RunArg, Stm, SS)
% LQR_CONTROL  LQR attitude control system for the Millennium Falcon.

%% Roll

% Local aliases
A = SS.Roll.A;
B = SS.Roll.B;
C = SS.Roll.C;
D = SS.Roll.D;

% Heuristically determined Q and R matrices.
Q = [100, 0;
     0,   1];
R = 0.05;

K = lqr(A, B, Q, R);

A_cl = A - B*K;
sys = ss(A_cl, B, C, D);

Lqr.Roll.Q    = Q;
Lqr.Roll.R    = R;
Lqr.Roll.K    = K;
Lqr.Roll.A_cl = A_cl;
Lqr.Roll.sys  = sys;

end