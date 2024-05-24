function Lqr = lqr_control(RunArg, Stm, SS)
% LQR_CONTROL  LQR attitude control system for the Millennium Falcon.

%% Roll

% Local aliases
A = SS.Roll.A;
B = SS.Roll.B;
C = SS.Roll.C;
D = SS.Roll.D;

% Heuristically determined Q and R matrices.
Q = diag([1, 0]);  % That's the roll angle that matters here.
R = 1e-7;

% Optimal gain
K = lqr(A, B, Q, R);

% Full state feedback system
A_cl = A - B*K;
sys = ss(A_cl, B, C, D);

timeSample = 0:0.01:Stm.Roll.settlingTime;
target = Stm.Roll.angle;

x0 = [target; 0];
[y, t, ~] = initial(sys, x0, timeSample);
figure("WindowStyle", "docked");
plot(t, 1 - y/target);

Lqr.Roll.Q    = Q;
Lqr.Roll.R    = R;
Lqr.Roll.K    = K;
Lqr.Roll.A_cl = A_cl;
Lqr.Roll.sys  = sys;


end