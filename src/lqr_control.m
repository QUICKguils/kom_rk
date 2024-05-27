function Lqr = lqr_control(RunArg, Stm, Reqr, SS)
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
%   Reqr   (struct) -- Requirements estimation
%   SS     (struct) -- State-space representations
% Return:
%   Lqr.Roll, Lqr.Pitch, Lqr.Yaw (struct) -- LQR controllers, with fields:
%     Q        (2x2 double) -- weight on the state vector
%     R        (double)     -- weight on the input vector
%     K        (1x2 double) -- optimal proportional gain
%     A_cl     (2x2 double) -- closed loop A matrix
%     sys      (2x1 ss)     -- proportional gain, closed loop system
%     refVec   (2x1 double) -- initial conditions of the system
%     tSample  (1xN double) -- time sample for the step response
%     response (1xN double) -- system response to a step

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

% 3. Compute the step responses

Lqr = compute_step_responses(Stm, Reqr, Lqr);

% 4. Check the performance requirements

check_reqr(Stm, Lqr);

% 5. Plot the step responses

if contains(RunArg.opts, 'p')
	plot_step_responses(Lqr);
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
		Q = diag([1e10, 1]);
		R = 1e-5;
	case 'pitch'
		Q = diag([1e10, 1]);
		R = 10^(-5.8);
	case 'yaw'
		Q = diag([1e10, 1]);
		R = 10^(-6.7);
end
end

%% 2. LQR controller for the state-space systems

function Lqr = lqr_system(SS_rot, Q, R)
% LQR_SYSTEM Return LQR controller for the given state-space system.

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


%% 3. Compute the step responses

function Lqr = compute_step_responses(Stm, Reqr, Lqr)
% COMPUTE_STEP_RESPONSES  Compute the step responses of the LQR controllers.

% Reference rotation vectors
Lqr.Roll.refVec  = [Stm.Roll.angle;       0];
Lqr.Pitch.refVec = [Stm.Pitch.angle;      0];
Lqr.Yaw.refVec   = [Reqr.YawEvo.devAngle; 0];

% Time samples for the plots
t0 = 0;  % Initial time [s].
Lqr.Roll.tSample  = linspace(t0, 1.75*Stm.Roll.settlingTime);
Lqr.Pitch.tSample = linspace(t0, 2   *Stm.Pitch.settlingTime);
Lqr.Yaw.tSample   = linspace(t0, 2.5 *Reqr.YawEvo.recovTime);
% NOTE:
% The weirds coeffs are just there to extend a bit the response after
% the settling time, so that we better see the convergence on the step
% plots.

% Responses of the LQR controller
[Lqr.Roll.response,  ~, ~] = initial(Lqr.Roll.sys,  Lqr.Roll.refVec,  Lqr.Roll.tSample);
[Lqr.Pitch.response, ~, ~] = initial(Lqr.Pitch.sys, Lqr.Pitch.refVec, Lqr.Pitch.tSample);
[Lqr.Yaw.response,   ~, ~] = initial(Lqr.Yaw.sys,   Lqr.Yaw.refVec,   Lqr.Yaw.tSample);
end

%% 4. Check the performance requirements

function check_reqr(Stm, Lqr)
% CHECK_REQR  Check that the step responses meets the requirements.

% Overshoot requirements
if any(Lqr.Roll.response >= (1 + Stm.Roll.percentageOvershoot/100))
	warning("Too much overshoot for LQR in Roll");
end
if any(Lqr.Pitch.response >= (1 + Stm.Pitch.percentageOvershoot/100))
	warning("Too much overshoot for LQR in Roll");
end

% Settling time requirements
%
% It can be verified on the plotted graph.
% TODO: implement that programatically

end


%% 5. Plot the step responses

function plot_step_responses(Lqr)
% PLOT_STEP_RESPONSES  Plot the step responses of the LQR controllers.

% Instantiate a new figure object
figure("WindowStyle", "docked");
title("Step responses of the LQR controllers");

% Plot the step responses
hold on;
plot(Lqr.Roll.tSample,  (1 -  Lqr.Roll.response(:, 1)/Lqr.Roll.refVec(1)));
plot(Lqr.Pitch.tSample, (1 - Lqr.Pitch.response(:, 1)/Lqr.Pitch.refVec(1)));
plot(Lqr.Yaw.tSample,   (1 -   Lqr.Yaw.response(:, 1)/Lqr.Yaw.refVec(1)));
hold off;
grid;
xlabel("Time (s)");
ylabel("Normalized rotation angle (deg)");
legend('Roll', 'Pitch', 'Yaw', 'location', 'southeast');
end
