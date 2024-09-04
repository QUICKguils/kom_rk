function Lqr = lqr_control(RunArg, Stm, Reqr, SS)
% LQR_CONTROL  LQR attitude control system for the Kom'rk.
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

Lqr = heuristic_QR();

% 2. LQR controller for the state-space systems

Lqr.Roll  = lqr_system(SS.Roll,  Lqr.Roll);
Lqr.Pitch = lqr_system(SS.Pitch, Lqr.Pitch);
Lqr.Yaw   = lqr_system(SS.Yaw,   Lqr.Yaw);

% 3. Compute the step responses

Lqr = compute_step_responses(Stm, Reqr, Lqr);

% 4. Check the performance requirements

check_reqr(Stm, Reqr, Lqr);

% 5. Plot the step responses

if contains(RunArg.opts, 'p')
	plot_responses(Stm, Reqr, Lqr);
end

end

%% 1. Heuristics for Q and R matrices

function Lqr = heuristic_QR()
% HEURISTIC_QR  Store the heuristic values of Q and R.

	Lqr.Roll.Q  = diag([1e2, 1e2]);
	Lqr.Roll.R  = 1e-13;
	
	Lqr.Pitch.Q = diag([1e3, 1e2]);
	Lqr.Pitch.R = 1e-13;

	Lqr.Yaw.Q   = diag([3e3, 2e3]);
	Lqr.Yaw.R   = 1e-13;
end

%% 2. LQR controller for the state-space systems

function Lqr_rot = lqr_system(SS_rot, Lqr_rot)
% LQR_SYSTEM Return LQR controller for the given state-space system.

% Local aliases
A = SS_rot.A;
B = SS_rot.B;
C_SISO = SS_rot.C_SISO;
D_SISO = SS_rot.D_SISO;
C_MIMO = SS_rot.C_MIMO;
D_MIMO = SS_rot.D_MIMO;

% Optimal gain
K = lqr(A, B, Lqr_rot.Q, Lqr_rot.R);

% Full state feedback system
A_cl = A - B*K;
sys_SISO = ss(A_cl, B, C_SISO, D_SISO);
sys_MIMO = ss(A_cl, B, C_MIMO, D_MIMO);

% Build the return data structure
Lqr_rot.K    = K;
Lqr_rot.A_cl = A_cl;
Lqr_rot.sys_SISO  = sys_SISO;
Lqr_rot.sys_MIMO  = sys_MIMO;
end

%% 3. Compute the step responses

function Lqr = compute_step_responses(Stm, Reqr, Lqr)
% COMPUTE_STEP_RESPONSES  Compute the step responses of the LQR controllers.

% Time samples for the plots
t0 = 0;  % Initial time [s].
Lqr.Roll.timeSample  = linspace(t0, 2*Stm.Roll.settlingTime);
Lqr.Pitch.timeSample = linspace(t0, 2*Stm.Pitch.settlingTime);
Lqr.Yaw.timeSample   = linspace(t0, 2*Reqr.YawEvo.recovTime);

% Responses of the LQR controller
%
% The solution is firstly obtained by considering the full state
% feedback system (i.e., where the reference signal is null) and
% imposing the target state as initial condition of the system. For roll
% and pitch, the response is then translated and flipped so that it
% corresponds to the step response of the system to the target state.

Lqr.Roll.targetState  = [Stm.Roll.angle;       0];
Lqr.Pitch.targetState = [Stm.Pitch.angle;      0];
Lqr.Yaw.initialState  = [Reqr.YawEvo.devAngle; 0];

[rollICResponse,  ~, ~] = initial(Lqr.Roll.sys_MIMO,  Lqr.Roll.targetState,  Lqr.Roll.timeSample);
[pitchICResponse, ~, ~] = initial(Lqr.Pitch.sys_MIMO, Lqr.Pitch.targetState, Lqr.Pitch.timeSample);
[yawICResponse,   ~, ~] = initial(Lqr.Yaw.sys_MIMO,   Lqr.Yaw.initialState,  Lqr.Yaw.timeSample);

Lqr.Roll.rotAngle  = Lqr.Roll.targetState(1) - rollICResponse(:, 1);
Lqr.Roll.rotRate   = Lqr.Roll.targetState(2) - rollICResponse(:, 2);
Lqr.Pitch.rotAngle = Lqr.Pitch.targetState(1) - pitchICResponse(:, 1);
Lqr.Pitch.rotRate  = Lqr.Pitch.targetState(2) - pitchICResponse(:, 2);
Lqr.Yaw.rotAngle   = yawICResponse(:, 1);
Lqr.Yaw.rotRate    = yawICResponse(:, 2);

% Armature voltages (control inputs): u = -K*x
%
% NOTE:
% - For roll and pitch, take the opposite, as the signal has been flipped.
% - Don't forget to divide per number of activated wheels, to get
%   voltage across ONE wheel only.
Lqr.Roll.voltage  = (+Lqr.Roll.K  * rollICResponse')'/2;
Lqr.Pitch.voltage = (+Lqr.Pitch.K * pitchICResponse')'/2;
Lqr.Yaw.voltage   = (-Lqr.Yaw.K   * yawICResponse')'/4;
end

%% 4. Check the performance requirements

function check_reqr(Stm, Reqr, Lqr)
% CHECK_REQR  Check that the step responses meets the requirements.

% Load locally the requirements bounds
Bounds = reqr_bounds(Stm, Reqr);

if ~Bounds.Roll.check(Lqr.Roll.timeSample, Lqr.Roll.rotAngle)
	warning("Roll step response does not meet the performance requirements.");
end
if ~Bounds.Pitch.check(Lqr.Pitch.timeSample, Lqr.Pitch.rotAngle)
	warning("Pitch step response does not meet the performance requirements.");
end
if ~Bounds.Yaw.check(Lqr.Yaw.timeSample, Lqr.Yaw.rotAngle)
	warning("Yaw step response does not meet the performance requirements.");
end
end

%% 5. Plot the step responses

function plot_responses(Stm, Reqr, Lqr)
% PLOT_responses  Plot the rotation and voltage profiles of the LQR controllers.

% Load locally the requirements bounds
Bounds = reqr_bounds(Stm, Reqr);

% Instantiate a new figure object
figure("WindowStyle", "docked");
sgtitle("Rotation and voltage profiles of the LQR controllers");

% Plot the step responses
subplot(2, 3, 1); hold on;
plot(Lqr.Roll.timeSample, rad2deg(Lqr.Roll.rotAngle));
plot(Bounds.Roll.Upper.time, rad2deg(Bounds.Roll.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Roll.Lower.time, rad2deg(Bounds.Roll.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Roll");
xlabel("Time (s)");
ylabel("Angle (°)");

subplot(2, 3, 2); hold on;
plot(Lqr.Pitch.timeSample, rad2deg(Lqr.Pitch.rotAngle));
plot(Bounds.Pitch.Upper.time, rad2deg(Bounds.Pitch.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Pitch.Lower.time, rad2deg(Bounds.Pitch.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Pitch");
xlabel("Time (s)");
ylabel("Angle (°)");

subplot(2, 3, 3); hold on;
plot(Lqr.Yaw.timeSample, rad2deg(Lqr.Yaw.rotAngle));
plot(Bounds.Yaw.Upper.time, rad2deg(Bounds.Yaw.Upper.angle), 'LineWidth', 1, 'Color', 'r');
plot(Bounds.Yaw.Lower.time, rad2deg(Bounds.Yaw.Lower.angle), 'LineWidth', 1, 'Color', 'r');
hold off; grid;
title("Yaw");
xlabel("Time (s)");
ylabel("Angle (°)");

% Plot the voltage inputs
subplot(2, 3, 4); hold on;
plot(Lqr.Roll.timeSample, Lqr.Roll.voltage);
hold off; grid;
xlabel("Time (s)");
ylabel("Voltage (V)");

subplot(2, 3, 5); hold on;
plot(Lqr.Pitch.timeSample, Lqr.Pitch.voltage);
hold off; grid;
xlabel("Time (s)");
ylabel("Voltage (V)");

subplot(2, 3, 6); hold on;
plot(Lqr.Yaw.timeSample, Lqr.Yaw.voltage);
hold off; grid;
xlabel("Time (s)");
ylabel("Voltage (V)");
end
