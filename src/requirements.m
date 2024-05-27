function Reqr = requirements(RunArg, Stm)
% REQUIREMENTS  Requirements estimation for the attitude control system.
%
% Arguments:
%   RunArg (struct) -- Code execution parameters, with fields:
%     opts  (1xN char) -- Output options
%       'p' -> Enable [P]lots creation
%   Stm    (struct) -- Project statement data
% Return:
%   Reqr (struct) -- Requirements estimation, with fields:
%     RollEvo  (struct) -- Evolution of attitude and RW currents in roll
%     PitchEvo (struct) -- Evolution of attitude and RW currents in pitch
%     YawEvo   (struct) -- Evolution of attitude and RW currents in yaw
%     Sizing   (struct) -- Reaction wheel sizing

% TODO:
% - Etimation of the wheels sizing

% Unpack relevant execution parameters
LocalRunArg = {RunArg.opts};
opts = LocalRunArg{:};

% 1. Time evolution laws

Reqr.RollEvo  = evolution_from_rest(Stm.Roll, Stm.Falcon.Ixx);
Reqr.PitchEvo = evolution_from_rest(Stm.Pitch, Stm.Falcon.Iyy);
Reqr.YawEvo   = evolution_for_yaw(Stm.Yaw, Stm.Falcon.Izz);

% 2. Electrical current estimates

[i_roll, i_pitch, i_yaw] = current_needs(Stm, Reqr);
Reqr.RollEvo.current  = i_roll;
Reqr.PitchEvo.current = i_pitch;
Reqr.YawEvo.current   = i_yaw;

% 3. Reaction wheels sizing

Reqr.Sizing = wheel_sizing(Stm, Reqr);

% 4. Requirements plot

if contains(opts, 'p')
	plot_requirements(Reqr);
end

end

%% 1. Time evolution laws

function RotEvo = evolution_from_rest(RotDesc, I)
% EVOLUTION_FROM_STEADY_STATE  Time evolutions from steady state.
%
% This function gives the time expressions for the torque, angular
% momentum and rotation angle assuming an initial rest condition.
% That is, the initial time is 0s, the inital rotation angle is 0 rad,
% and the inital rotation speed is 0 rad/s.
%
% See formula derivations in the report.

% Constants and local aliases
ti = 0;  % Initial time [s]
tf = RotDesc.settlingTime;

% Constant moment, that yield half the rotation angle
% at half the rotation time
Mc = RotDesc.angle/(tf/2)^2 * I;

% Time expressions of the torque, angular momentum and roll angle
M     = @(t) Mc                              .* (ti<=t)  .* (t<=tf/2)...
	       - Mc                              .* (t>tf/2) .* (t<=tf);
H     = @(t) Mc*t                            .* (ti<=t)  .* (t<=tf/2)...
	       + Mc*(tf-t)                       .* (t>tf/2) .* (t<=tf);
angle = @(t) Mc/(2*I)*t.^2                   .* (ti<=t)  .* (t<=tf/2)...
	       + Mc/(2*I)*(-t.^2+2*tf*t-tf.^2/2) .* (t>tf/2) .* (t<=tf);

% Build the return data structure
RotEvo.duration = tf - ti;
RotEvo.torque   = M;
RotEvo.momentum = H;
RotEvo.angle    = angle;
end

function YawEvo = evolution_for_yaw(YawDesc, Izz)
% EVOLUTION_FOR_YAW  Time evolution for the yaw requirement.

% Hit phase: laser hit + stopping yaw speed
HitDesc.settlingTime = 2 * YawDesc.laserTime;
HitDesc.angle = YawDesc.laserTorque/Izz * YawDesc.laserTime^2;
HitEvo = evolution_from_rest(HitDesc, Izz);

% Recovery phase: bring back yaw angle to 0 rad
RecovDesc.settlingTime = YawDesc.recoveryTime - YawDesc.laserTime;
RecovDesc.angle = -HitDesc.angle;
RecovEvo = evolution_from_rest(RecovDesc, Izz);

% Combine the yaw attitude of hit and recovery phase
YawEvo.duration = HitEvo.duration + RecovEvo.duration;
YawEvo.torque   = @(t) HitEvo.torque(t)   + RecovEvo.torque(t-HitEvo.duration);
YawEvo.momentum = @(t) HitEvo.momentum(t) + RecovEvo.momentum(t-HitEvo.duration);
YawEvo.angle    = @(t) HitEvo.angle(t)    + RecovEvo.angle(t-HitEvo.duration) ...
	                 + HitDesc.angle * (t > HitEvo.duration) .* (t <= YawEvo.duration);
end

%% 2. Electrical current estimates

function [i_roll, i_pitch, i_yaw] = current_needs(Stm, Reqr)
% CURRENT_NEEDS  Compute the current in RW to achieve required attitude.
%
% See formula derivations in the report.

i_roll  = @(t) Reqr.RollEvo.torque(t)  ./ (2*Stm.RW.torqueCst*sin(Stm.RW.beta));
i_pitch = @(t) Reqr.PitchEvo.torque(t) ./ (2*Stm.RW.torqueCst*sin(Stm.RW.beta));
i_yaw   = @(t) Reqr.YawEvo.torque(t)   ./ (4*Stm.RW.torqueCst*cos(Stm.RW.beta));
end

%% 3. Reaction wheels sizing

function Sizing = wheel_sizing(Stm, Reqr, varargin)
% WHEEL_SIZING  Propose a first sizing for the reaction wheels.
%
% This function assumes that the reaction wheel moments of inertia are
% similar to that of a homogeneous disk.
%
% Optional arguments:
%   rwSpeedMargin (double) -- default: 20%
%     Percentage decrease of the maximum rotation speed allowed for the
%     reaction wheels.
%   heightGuess (double) -- default: 0.5m
%     Value of the disk height that is chosen a priori for the reaction
%     wheels.

% Set default value for optional inputs
optargs = {20, 0.5};
% Overwrite default value of optional inputs
optargs(1:numel(varargin)) = varargin;
% Place optional args in memorable variable names
[rwSpeedMargin, heightGuess] = optargs{:};

% Find the maximum angular momentum that the wheel must provide
% Sadly, Matlab don't offer a `fmaxbnd` function
[~, invMaxMomentumRoll]  = fminbnd(@(t) -Reqr.RollEvo.momentum(t),  0, Reqr.RollEvo.duration);
[~, invMaxMomentumPitch] = fminbnd(@(t) -Reqr.PitchEvo.momentum(t), 0, Reqr.PitchEvo.duration);
[~, invMaxMomentumYaw]   = fminbnd(@(t) -Reqr.YawEvo.momentum(t),   0, Reqr.YawEvo.duration);

% Apply the security factor to the wheel maximum rotation speed
designSpeed = (1 - rwSpeedMargin/100) * Stm.RW.maxSpeed;

% Minimal moment of inertia required for the RW
IrwRoll  = -invMaxMomentumRoll  / (2*designSpeed*sin(Stm.RW.beta));
IrwPitch = -invMaxMomentumPitch / (2*designSpeed*sin(Stm.RW.beta));
IrwYaw   = -invMaxMomentumYaw   / (4*designSpeed*cos(Stm.RW.beta));
Irw = max([IrwRoll, IrwPitch, IrwYaw]);

% Radius needed to reach this minimal moment of inertia
radius = (2*Irw/(pi*Stm.RW.density*heightGuess))^(1/4);

% Build return data structure
Sizing.designSpeed    = designSpeed;
Sizing.designSpeedRpm = designSpeed * 30/pi;
Sizing.radius         = radius;
Sizing.height         = heightGuess;
Sizing.Irw            = Irw;
end

%% 4. Requirements plot

function Reqr = plot_requirements(Reqr)
% PLOT_REQUIREMENTS  Plot the attitude and current that meets the requirements.

% Build time samples for the plots
nSample = 101;
Reqr.RollEvo.tSample  = linspace(0, Reqr.RollEvo.duration,  nSample);
Reqr.PitchEvo.tSample = linspace(0, Reqr.PitchEvo.duration, nSample);
Reqr.YawEvo.tSample   = linspace(0, Reqr.YawEvo.duration,   nSample);

% Instantiate a new figure object
figure("WindowStyle", "docked");
title("Time evolutions that satisfy the requirements");

% Torques
subplot(2, 2, 1);
hold on;
plot(Reqr.RollEvo.tSample,  Reqr.RollEvo.torque(Reqr.RollEvo.tSample));
plot(Reqr.PitchEvo.tSample, Reqr.PitchEvo.torque(Reqr.PitchEvo.tSample));
plot(Reqr.YawEvo.tSample,   Reqr.YawEvo.torque(Reqr.YawEvo.tSample));
grid;
xlabel("Time (s)");
ylabel("Torque (N*m)");
% legend('Roll', 'Pitch', 'Yaw');

% Angular moments
subplot(2, 2, 2);
hold on
plot(Reqr.RollEvo.tSample,  Reqr.RollEvo.momentum(Reqr.RollEvo.tSample));
plot(Reqr.PitchEvo.tSample, Reqr.PitchEvo.momentum(Reqr.PitchEvo.tSample));
plot(Reqr.YawEvo.tSample,   Reqr.YawEvo.momentum(Reqr.YawEvo.tSample));
grid;
xlabel("Time (s)");
ylabel("Momentum (kg*m²/s)");
% legend('Roll', 'Pitch', 'Yaw');

% Rotation angles
subplot(2, 2, 3);
hold on
plot(Reqr.RollEvo.tSample,  rad2deg(Reqr.RollEvo.angle(Reqr.RollEvo.tSample)));
plot(Reqr.PitchEvo.tSample, rad2deg(Reqr.PitchEvo.angle(Reqr.PitchEvo.tSample)));
plot(Reqr.YawEvo.tSample,   rad2deg(Reqr.YawEvo.angle(Reqr.YawEvo.tSample)));
grid;
xlabel("Time (s)");
ylabel("Rotation angle (deg)");
% legend('Roll', 'Pitch', 'Yaw');

% Electrical current profile
subplot(2, 2, 4);
hold on
plot(Reqr.RollEvo.tSample,  Reqr.RollEvo.current(Reqr.RollEvo.tSample));
plot(Reqr.PitchEvo.tSample, Reqr.PitchEvo.current(Reqr.PitchEvo.tSample));
plot(Reqr.YawEvo.tSample,   Reqr.YawEvo.current(Reqr.YawEvo.tSample));
grid;
xlabel("Time (s)");
ylabel("Current (A)");
% legend('Roll', 'Pitch', 'Yaw');

hold off;
end
