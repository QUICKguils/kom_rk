function Lqr = lqr_control(RunArg, Stm, SS)
% LQR_CONTROL  LQR attitude control system for the Millennium Falcon.

[A, B, C, D] = get_roll_ss(Stm);

% Heuristically determined Q and R matrices.
Q = [100, 0;
     0,   1];
R = 0.05;

K = lqr(A, B, Q, R);

A_cl = A - B*K;
sys = ss(A_cl, B, C, D);

end