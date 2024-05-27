function Pid = pid_control(RunArg, Stm, SS)
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

% 1. Heuristics for P, I and D terms

% 2. PID controller for the state-space systems

% 3. Plot the step responses

if contains(RunArg.opts, 'p')
	plot_step_responses(Stm, Lqr);
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

function Lqr = lqr_system(SS_rot, P, I, D)
% PID_SYSTEM Return PID controller for the given state-space system.

end

%% 3. Plot the step responses

function plot_step_responses(Stm, Lqr)
% PLOT_STEP_RESPONSES  Plot the step responses of the PID controllers.

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
title("Step responses of the PID controllers");

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
