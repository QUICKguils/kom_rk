function Pid = pid_control(RunArg, Stm, Reqr, SS)
% PID_CONTROL  PID attitude control system for the Kom'rk.
%
% Arguments:
%   RunArg (struct) -- Code execution parameters, with field:
%   opts (1xN char) -- Output options
%     'p' -> Enable [P]lots creation
%   selsim ({'roll', 'pitch', 'yaw'}) -- Select simulink model
%     'roll'  -> Run the roll models
%     'pitch' -> Run the pitch models
%     'yaw'   -> Run the yaw models
%   runsim (bool) -- Run the LQR and PID simulink models
%     true  -> Run the selected simulink models.
%     false -> Do not activate simulink.
%   Stm (struct) -- Project statement data
%   Reqr (struct) -- Requirements estimation
%   SS (struct) -- State-space representations
% Return:
%   Pid.Roll, Pid.Pitch, Pid.Yaw (struct) -- PID controllers data.

% 1. Heuristics for P, I and D terms

Pid = heuristic_PID();

% 2. PID controller for the state-space systems

% 3. Compute the step responses

% Lqr = compute_step_responses(Stm, Reqr, Pid);

% 4. Check the performance requirements

% check_reqr(Stm, Pid);

% 5. Plot the step responses

if contains(RunArg.opts, 'p')
	plot_responses(Stm, Reqr, Pid);
end

% 6. Run the Simulink models

if RunArg.runsim
	% Make sure that the project data are loaded in the workspace
	assignin("base", "RunArg", RunArg);
	assignin("base", "Stm", Stm);
	assignin("base", "Reqr", Reqr);
	assignin("base", "SS", SS);
	assignin("base", "Pid", Pid);

	Pid = run_simulink_models(Pid);

	if contains(RunArg.opts, 'p')
		plot_rw_speeds(Pid);
	end
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
	warning("PID roll maneuver does not meet the performance requirements.");
end
if ~Bounds.Pitch.check(Pid.Pitch.tSample, Pid.Pitch.response)
	warning("PID pitch maneuver does not meet the performance requirements.");
end
if ~Bounds.Yaw.check(Pid.Yaw.tSample, Pid.Yaw.response)
	warning("PID yaw maneuver does not meet the performance requirements.");
end
end

%% 5. Plot the step responses

function plot_responses(Stm, Reqr, Pid)
% PLOT_RESPONSES  Plot the rotation and voltage profiles of the PID controllers.

% Load locally the requirements bounds
Bounds = reqr_bounds(Stm, Reqr);

% Instantiate a new figure object
figure("WindowStyle", "docked");
sgtitle("Rotation and voltage profiles of the PID controllers");

% Plot the step responses
subplot(2, 3, 1); hold on;
plot(Pid.Roll.timeSample, rad2deg(Pid.Roll.rotAngle));
plot(Bounds.Roll.Upper.time, rad2deg(Bounds.Roll.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Roll.Lower.time, rad2deg(Bounds.Roll.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Roll");
xlabel("Time (s)");
ylabel("Angle (°)");

subplot(2, 3, 2); hold on;
plot(Pid.Pitch.timeSample, rad2deg(Pid.Pitch.rotAngle));
plot(Bounds.Pitch.Upper.time, rad2deg(Bounds.Pitch.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Pitch.Lower.time, rad2deg(Bounds.Pitch.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Pitch");
xlabel("Time (s)");
ylabel("Angle (°)");

subplot(2, 3, 3); hold on;
plot(Pid.Yaw.timeSample, rad2deg(Pid.Yaw.rotAngle));
plot(Bounds.Yaw.Upper.time, rad2deg(Bounds.Yaw.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Yaw.Lower.time, rad2deg(Bounds.Yaw.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Yaw");
xlabel("Time (s)");
ylabel("Angle (°)");

% Plot the voltage inputs
subplot(2, 3, 4); hold on;
plot(Pid.Roll.timeSample, Pid.Roll.voltage);
hold off; grid;
xlabel("Time (s)");
ylabel("Voltage (V)");

subplot(2, 3, 5); hold on;
plot(Pid.Pitch.timeSample, Pid.Pitch.voltage);
hold off; grid;
xlabel("Time (s)");
ylabel("Voltage (V)");

subplot(2, 3, 6); hold on;
plot(Pid.Yaw.timeSample, Pid.Yaw.voltage);
hold off; grid;
xlabel("Time (s)");
ylabel("Voltage (V)");
end

%% 6. Run the Simulink models

function Pid = run_simulink_models(Pid)
% RUN_SIMULINK_MODELS  Run simulink models and save scope data.

	evalin('base','RunArg.selsim = ''roll'';');
	outRoll = sim("src\slx\pid_control_model.slx");
	Pid.Roll.SimulinkScopes.time     = outRoll.tout;
	Pid.Roll.SimulinkScopes.voltage  = outRoll.voltage.signals.values;
	Pid.Roll.SimulinkScopes.current  = outRoll.current.signals.values;
	Pid.Roll.SimulinkScopes.rotAngle = outRoll.rotAngle.signals.values;
	Pid.Roll.SimulinkScopes.rotRate  = outRoll.rotRate.signals.values;
	Pid.Roll.SimulinkScopes.rwSpeed  = outRoll.rwSpeed.signals.values;

	evalin('base','RunArg.selsim = ''pitch'';');
	outPitch = sim("src\slx\pid_control_model.slx");
	Pid.Pitch.SimulinkScopes.time     = outPitch.tout;
	Pid.Pitch.SimulinkScopes.voltage  = outPitch.voltage.signals.values;
	Pid.Pitch.SimulinkScopes.current  = outPitch.current.signals.values;
	Pid.Pitch.SimulinkScopes.rotAngle = outPitch.rotAngle.signals.values;
	Pid.Pitch.SimulinkScopes.rotRate  = outPitch.rotRate.signals.values;
	Pid.Pitch.SimulinkScopes.rwSpeed  = outPitch.rwSpeed.signals.values;

	evalin('base','RunArg.selsim = ''yaw'';');
	outYaw = sim("src\slx\pid_control_model.slx");
	Pid.Yaw.SimulinkScopes.time     = outYaw.tout;
	Pid.Yaw.SimulinkScopes.voltage  = outYaw.voltage.signals.values;
	Pid.Yaw.SimulinkScopes.current  = outYaw.current.signals.values;
	Pid.Yaw.SimulinkScopes.rotAngle = outYaw.rotAngle.signals.values;
	Pid.Yaw.SimulinkScopes.rotRate  = outYaw.rotRate.signals.values;
	Pid.Yaw.SimulinkScopes.rwSpeed  = outYaw.rwSpeed.signals.values;
end

function plot_rw_speeds(Pid)
% PLOT_RW_SPEEDS  Plot the RW speeds, obtained from the Simulink models.

	% Instantiate a new figure object
	figure("WindowStyle", "docked");
	sgtitle("RW speeds (Simulink model for PID)");
	
	subplot(1, 3, 1);
	plot(Pid.Roll.SimulinkScopes.time, Pid.Roll.SimulinkScopes.rwSpeed);
	grid;
	title("Roll");
	xlabel("Time (s)");
	ylabel("Speed (rpm)");
	
	subplot(1, 3, 2);
	plot(Pid.Pitch.SimulinkScopes.time, Pid.Pitch.SimulinkScopes.rwSpeed);
	grid;
	title("Pitch");
	xlabel("Time (s)");
	ylabel("Speed (rpm)");
	
	subplot(1, 3, 3);
	plot(Pid.Yaw.SimulinkScopes.time, Pid.Yaw.SimulinkScopes.rwSpeed);
	grid;
	title("Yaw");
	xlabel("Time (s)");
	ylabel("Speed (rpm)");
end