function Pid = pid_control(RunArg, Stm, Reqr, SS)
% PID_CONTROL  PID attitude control system for the Millennium Falcon.
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
%   Pid.Roll, Pid.Pitch, Pid.Yaw (struct) -- PID controllers, with fields:
%     P (array[double]) -- Proportional term of C(s).
%     I (array[double]) -- Integral     term of C(s).
%     D (array[double]) -- Derivative   term of C(s).

% TODO:
% - run selected simulink model by Runarg.selsim

Pid.test = 0;

% 1. Heuristics for P, I and D terms

% 2. PID controller for the state-space systems

% 3. Compute the step responses

% Lqr = compute_step_responses(Stm, Reqr, Pid);

% 4. Check the performance requirements

% check_reqr(Stm, Pid);

% 5. Plot the step responses

if contains(RunArg.opts, 'p')
% 	plot_step_responses(Lqr);
end
end

%% 1. Heuristics for P, I and D terms

function [P, I, D] = heuristic_PID(axis)
% HEURISTIC_PID  Store the heuristic values of P, D and I.
%
% Argument:
%   axis ({'roll', 'pitch', 'yaw'}) -- select the s.s. system

% TODO: find good PID values with the aid of sisotool

switch axis
	case 'roll'
		P = 1;
		I = 1;
		D = 1;
	case 'pitch'
		P = 1;
		I = 1;
		D = 1;
	case 'yaw'
		P = 1;
		I = 1;
		D = 1;
end
end

%% 2. PID controller for the state-space systems

function Pid = pid_system(SS_rot, P, I, D)
% PID_SYSTEM Return PID controller for the given state-space system.

end

%% 3. Compute the step responses

function Pid = compute_step_responses(Stm, Reqr, Pid)
% COMPUTE_STEP_RESPONSES  Compute the step responses of the PID controllers.

% Reference rotation vectors
Pid.Roll.refVec  = [Stm.Roll.angle;       0];
Pid.Pitch.refVec = [Stm.Pitch.angle;      0];
Pid.Yaw.refVec   = [Reqr.YawEvo.devAngle; 0];

% Time samples for the plots
t0 = 0;  % Initial time [s].
Pid.Roll.tSample  = linspace(t0, 1.75*Stm.Roll.settlingTime);
Pid.Pitch.tSample = linspace(t0, 2   *Stm.Pitch.settlingTime);
Pid.Yaw.tSample   = linspace(t0, 2.5 *Reqr.YawEvo.recovTime);
% NOTE:
% The weirds coeffs are just there to extend a bit the response after
% the settling time, so that we better see the convergence on the step
% plots.

% Responses of the LQR controller
[Pid.Roll.response,  ~, ~] = initial(Pid.Roll.sys,  Pid.Roll.refVec,  Pid.Roll.tSample);
[Pid.Pitch.response, ~, ~] = initial(Pid.Pitch.sys, Pid.Pitch.refVec, Pid.Pitch.tSample);
[Pid.Yaw.response,   ~, ~] = initial(Pid.Yaw.sys,   Pid.Yaw.refVec,   Pid.Yaw.tSample);
end

%% 4. Check the performance requirements

function check_reqr(Stm, Pid)
% CHECK_REQR  Check that the step responses meets the requirements.

% Overshoot requirements
if any(Pid.Roll.response >= (1 + Stm.Roll.percentageOvershoot/100))
	warning("Too much overshoot for LQR in Roll");
end
if any(Pid.Pitch.response >= (1 + Stm.Pitch.percentageOvershoot/100))
	warning("Too much overshoot for LQR in Roll");
end

% Settling time requirements
%
% It can be verified on the plotted graph.
% TODO: implement that programatically

end


%% 5. Plot the step responses

function plot_step_responses(Pid)
% PLOT_STEP_RESPONSES  Plot the step responses of the PID controllers.

% Instantiate a new figure object
figure("WindowStyle", "docked");
title("Step responses of the LQR controllers");

% Plot the step responses
hold on;
plot(Pid.Roll.tSample,  (1 -  Pid.Roll.response(:, 1)/Pid.Roll.refVec(1)));
plot(Pid.Pitch.tSample, (1 - Pid.Pitch.response(:, 1)/Pid.Pitch.refVec(1)));
plot(Pid.Yaw.tSample,   (1 -   Pid.Yaw.response(:, 1)/Pid.Yaw.refVec(1)));
hold off;
grid;
xlabel("Time (s)");
ylabel("Normalized rotation angle (deg)");
legend('Roll', 'Pitch', 'Yaw', 'location', 'southeast');
end
