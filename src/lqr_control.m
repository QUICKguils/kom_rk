function Lqr = lqr_control(RunArg, Stm, SS)
% LQR_CONTROL  LQR attitude control system for the Millennium Falcon.
%
% Arguments:
%   RunArg (struct) -- Code execution parameters, with fields:
%     opts  (1xN char) -- Output options
%       'p' -> Enable [P]lots creation
%   Stm    (struct) -- Project statement data
%   SS     (struct) -- State-space representations
% Return:
%   Reqr (struct) -- Requirements estimation, with fields:
%     RollEvo  (struct) -- Evolution of torque, momentum and angle in roll
%     PitchEvo (struct) -- Evolution of torque, momentum and angle in pitch
%     YawEvo   (struct) -- Evolution of torque, momentum and angle in yaw

% Unpack relevant execution parameters
LocalRunArg = {RunArg.opts, Runarg.selsim};
[opts, selsim] = LocalRunArg{:};

%% Roll

% Local aliases
A = SS.Roll.A;
B = SS.Roll.B;
C = SS.Roll.C;
D = SS.Roll.D;

% Heuristically determined Q and R matrices.
Q = 1e13 * diag([1, 1]);  % The rotation angle matters
R = 0.001;

% Optimal gain
K = lqr(A, B, Q, R);

% Full state feedback system
A_cl = A - B*K;
sys = ss(A_cl, B, C, D);

% timeSample = 0:0.01:Stm.Roll.settlingTime;
% target = Stm.Roll.angle;
% 
% x0 = [target; 0];
% [y, t, ~] = initial(sys, x0, timeSample);
% % figure("WindowStyle", "docked");
% % plot(t, 1 - y/target);

Lqr.Roll.Q    = Q;
Lqr.Roll.R    = R;
Lqr.Roll.K    = K;
Lqr.Roll.A_cl = A_cl;
Lqr.Roll.sys  = sys;

% Simulink

Lqr.selectSim = 'roll';

end
