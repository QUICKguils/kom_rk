function Lqr = lqr_control(RunArg, Stm, SS)
% LQR_CONTROL  LQR attitude control system for the Millennium Falcon.
%
% Arguments:
%   RunArg (struct) -- Code execution parameters, with field:
%     opts  (1xN char) -- Output options
%       'p' -> Enable [P]lots creation
%     selsim ({'roll', 'pitch', 'yaw'}) -- Select simulink model
%       'roll'  -> Run the roll models
%       'pitch' -> Run the pitch models
%       'yaw'   -> Run the yaw models
%   Stm    (struct) -- Project statement data
%   SS     (struct) -- State-space representations
% Return:
%   Lqr.Roll, Lqr.Pitch, Lqr.Yaw (struct) -- LQR controllers, with fields:
%     Q (array[double]) -- Q matrix: weight on the state vector.
%     R (array[double]) -- R matrix: weight on the input vector.
%     K (array[double]) -- K matrix: optimal proportional gain.

% TODO:
% - run selected simulink model by Runarg.selsim

% 1. Heuristics for Q and R matrices

[Q_roll,  R_roll]  = heuristic_QR('roll');
[Q_pitch, R_pitch] = heuristic_QR('pitch');
[Q_yaw,   R_yaw]   = heuristic_QR('yaw');

% 2. LQR controller for the state-space systems

Lqr.Roll  = lqr_system(SS.Roll,  Q_roll,  R_roll);
Lqr.Pitch = lqr_system(SS.Pitch, Q_pitch, R_pitch);
Lqr.Yaw   = lqr_system(SS.Yaw,   Q_yaw,   R_yaw);

% 3. Plot the step responses

if contains(RunArg.opts, 'p')
	plot_step_responses(Stm, Lqr);
end

end

%% 1. Heuristics for Q and R matrices

function [Q, R] = heuristic_QR(axis)
% HEURISTIC_QR  Store the heuristic values of Q and R.
%
% Argument:
%   axis ({'roll', 'pitch', 'yaw'}) -- select the s.s. system

switch axis
	case 'roll'
		Q = 1e15*diag([1, 1]);
		R = 1;
	case 'pitch'
		Q = 1e18*diag([1, 1]);
		R = 1;
	case 'yaw'
		Q = 1e8*diag([1, 1]);
		R = 1;
end
end

%% 2. LQR controller for the state-space systems

function Lqr = lqr_system(SS_rot, Q, R)
% LQR_system Return LQR controller for the given state-space system.
%
% Arguments:

% Local aliases
A = SS_rot.A;
B = SS_rot.B;
C = SS_rot.C;
D = SS_rot.D;

% Optimal gain
K = lqr(A, B, Q, R);

% TODO: see if useful to compute A_cl and the close loop system
% % Full state feedback system
A_cl = A - B*K;
sys  = ss(A_cl, B, C, D);

% Build the return data structure
Lqr.Q    = Q;
Lqr.R    = R;
Lqr.K    = K;
Lqr.A_cl = A_cl;
Lqr.sys  = sys;
end

%% 3. Plot the step responses

function plot_step_responses(Stm, Lqr)
% PLOT_STEP_RESPONSES  Plot the step responses of the LQR controllers.

% Reference rotation vectors
refRoll  = [Stm.Roll.angle; 0];
refPitch = [Stm.Pitch.angle; 0];
refYaw   = [Stm.Pitch.angle; 0];  % TODO: change

% Time samples for the plots
t0 = 0;  % Initial time [s].
tSampleRoll  = linspace(t0, 1.5*Stm.Roll.settlingTime);
tSamplePitch = linspace(t0, 1.5*Stm.Pitch.settlingTime);
tSampleYaw   = linspace(t0, 1.5*Stm.Pitch.settlingTime); % TODO: change

% Responses of the LQR controller
[yRoll,  ~, ~] = initial(Lqr.Roll.sys,  refRoll,  tSampleRoll);
[yPitch, ~, ~] = initial(Lqr.Pitch.sys, refPitch, tSamplePitch);
[yYaw,   ~, ~] = initial(Lqr.Yaw.sys,   refYaw,   tSampleYaw);

% Instantiate a new figure object
figure("WindowStyle", "docked");
title("Step responses of the LQR controllers");

% Plot the step responses
hold on;
plot(tSampleRoll,  (1 -  yRoll(:, 1)/refRoll(1)));
plot(tSamplePitch, (1 - yPitch(:, 1)/refPitch(1)));
plot(tSampleYaw,   (1 - yYaw(:, 1)/refYaw(1)));
grid;
xlabel("Time (s)");
ylabel("Normalized rotation angle (deg)");
legend('Roll', 'Pitch', 'Yaw', 'location', 'southeast');
end