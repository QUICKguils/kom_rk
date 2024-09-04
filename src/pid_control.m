function Pid = pid_control(RunArg, Stm, Reqr, SS)
% PID_CONTROL  PID attitude control system for the Kom'rk.
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

function Pid = heuristic_PID()
% HEURISTIC_PID  Store the heuristic values of P, D and I.

		Pid.Roll.P = 1;
		Pid.Roll.I = 1;
		Pid.Roll.D = 1;

		Pid.Pitch.P = 1;
		Pid.Pitch.I = 1;
		Pid.Pitch.D = 1;

		Pid.Yaw.P = 1;
		Pid.Yaw.I = 1;
		Pid.Yaw.D = 1;
end

%% 2. PID controller for the state-space systems

function Pid = pid_system(SS_rot, Pid_rot)
% PID_SYSTEM Return PID controller for the given state-space system.

end

%% 3. Compute the step responses

function Pid = compute_step_responses(Stm, Reqr, Pid)
% COMPUTE_STEP_RESPONSES  Compute the step responses of the PID controllers.

% Time samples for the plots
t0 = 0;  % Initial time [s].
Pid.Roll.tSample  = linspace(t0, 2*Stm.Roll.settlingTime);
Pid.Pitch.tSample = linspace(t0, 2*Stm.Pitch.settlingTime);
Pid.Yaw.tSample   = linspace(t0, 2*Reqr.YawEvo.recovTime);

% Responses of the LQR controller
% NOTE: 

rollInitialState  = [Stm.Roll.angle;       0];
pitchInitialState = [Stm.Pitch.angle;      0];
yawInitialState   = [Reqr.YawEvo.devAngle; 0];

[rollResponse,  ~, ~] = initial(Lqr.Roll.sys,  rollInitialState,  Lqr.Roll.tSample);
[pitchResponse, ~, ~] = initial(Lqr.Pitch.sys, pitchInitialState, Lqr.Pitch.tSample);
[yawResponse,   ~, ~] = initial(Lqr.Yaw.sys,   yawInitialState,   Lqr.Yaw.tSample);

Pid.Roll.response  = rollInitialState(1)  - rollResponse(:, 1);
Pid.Pitch.response = pitchInitialState(1) - pitchResponse(:, 1);
Pid.Yaw.response   = yawResponse(:, 1);
end


%% 4. Check the performance requirements

function check_reqr(Stm, Reqr, Pid)
% CHECK_REQR  Check that the step responses meets the requirements.

% Load locally the requirements bounds
Bounds = reqr_bounds(Stm, Reqr);

if ~Bounds.Roll.check(Pid.Roll.tSample, Pid.Roll.response)
	warning("Roll step response does not meet the performance requirements.");
end
if ~Bounds.Pitch.check(Pid.Pitch.tSample, Pid.Pitch.response)
	warning("Pitch step response does not meet the performance requirements.");
end
if ~Bounds.Yaw.check(Pid.Yaw.tSample, Pid.Yaw.response)
	warning("Yaw step response does not meet the performance requirements.");
end
end

%% 5. Plot the step responses

function plot_step_responses(Stm, Reqr, Lqr)
% PLOT_STEP_RESPONSES  Plot the step responses of the LQR controllers.

% Load locally the requirements bounds
Bounds = reqr_bounds(Stm, Reqr);

% Instantiate a new figure object
figure("WindowStyle", "docked");
sgtitle("Step responses of the LQR controllers");

% Plot the step responses
subplot(1, 3, 1); hold on;
plot(Lqr.Roll.tSample, rad2deg(Lqr.Roll.response));
plot(Bounds.Roll.Upper.time, rad2deg(Bounds.Roll.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Roll.Lower.time, rad2deg(Bounds.Roll.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Roll");
xlabel("Time (s)");
ylabel("Angle (°)");

subplot(1, 3, 2); hold on;
plot(Lqr.Pitch.tSample, rad2deg(Lqr.Pitch.response));
plot(Bounds.Pitch.Upper.time, rad2deg(Bounds.Pitch.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Pitch.Lower.time, rad2deg(Bounds.Pitch.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Pitch");
xlabel("Time (s)");
ylabel("Angle (°)");

subplot(1, 3, 3); hold on;
plot(Lqr.Yaw.tSample, rad2deg(Lqr.Yaw.response));
plot(Bounds.Yaw.Upper.time, rad2deg(Bounds.Yaw.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Yaw.Lower.time, rad2deg(Bounds.Yaw.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Yaw");
xlabel("Time (s)");
ylabel("Angle (°)");
end