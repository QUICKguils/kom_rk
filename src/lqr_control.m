function Lqr = lqr_control(RunArg, SS)
% LQR_CONTROL  LQR attitude control system for the Millennium Falcon.
%
% Arguments:
%   RunArg (struct) -- Code execution parameters, with field:
%     selsim ({'roll', 'pitch', 'yaw'}) -- Select simulink model
%       'roll'  -> Run the roll models
%       'pitch' -> Run the pitch models
%       'yaw'   -> Run the yaw models
%   SS     (struct) -- State-space representations
% Return:
%   Lqr.Roll, Lqr.Pitch, Lqr.Yaw (struct) -- LQR controllers, with fields:
%     Q (array[double]) -- Q matrix: weight on the state vector.
%     R (array[double]) -- R matrix: weight on the input vector.
%     K (array[double]) -- K matrix: optimal proportional gain.

% TODO:
% - run selected simulink model by Runarg.selsim

% Unpack relevant execution parameters
LocalRunArg = {RunArg.selsim};
selsim = LocalRunArg{:};

% 1. Heuristics for Q and R matrices

[Q_roll,  R_roll]  = heuristic_QR('roll');
[Q_pitch, R_pitch] = heuristic_QR('pitch');
[Q_yaw,   R_yaw]   = heuristic_QR('yaw');

% 2. LQR controller for the state-space systems

Lqr.Roll  = lqr_system(SS.Roll,  Q_roll,  R_roll);
Lqr.Pitch = lqr_system(SS.Pitch, Q_pitch, R_pitch);
Lqr.Yaw   = lqr_system(SS.Yaw,   Q_yaw,   R_yaw);
end

%% 1. Heuristics for Q and R matrices

function [Q, R] = heuristic_QR(axis)
% HEURISTIC_QR  Return the heuristics values of Q and R.
%
% Argument:
%   axis ({'roll', 'pitch', 'yaw'}) -- select the axis of rotation

switch axis
	case 'roll'
		Q = 1e13 * diag([1, 1]);
		R = 0.001;
	case 'pitch'
		Q = 1e13 * diag([1, 1]);
		R = 0.001;
	case 'yaw'
		Q = 1e13 * diag([1, 1]);
		R = 0.001;
end
end

% 2. LQR controller for the state-space systems

function Lqr = lqr_system(SS_rot, Q, R)
% LQR_system Return LQR controller for the given state-space system.
%
% Arguments:

% Local aliases
A = SS_rot.A;
B = SS_rot.B;

% Optimal gain
K = lqr(A, B, Q, R);

% % Full state feedback system
% A_cl = A - B*K;  % TODO: see if useful to compute that
% sys = ss(A_cl, B, C, D);  % TODO: see if useful to compute that

% Build the return data structure
Lqr.Q    = Q;
Lqr.R    = R;
Lqr.K    = K;
% Lqr.A_cl = A_cl;
% Lqr.sys  = sys;
end
