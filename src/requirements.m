function Reqr = requirements(RunArg, Stm)
% REQUIREMENTS  Requirements estimation for the attitude control system.
%
% Arguments:
%   Stm    (struct) -- Project statement data
%   RunArg (struct) -- Code execution parameters, with fields:
%     opts  (1xN char) -- Output options
%       'p' -> Enable [P]lots creation
% Return:
%   Reqr (struct) -- Requirements estimation, with fields:
%     RollEvo  (struct) -- Evolution of torque, momentum and angle in roll.
%     PitchEvo (struct) -- Evolution of torque, momentum and angle in pitch.
%     YawEvo   (struct) -- Evolution of torque, momentum and angle in yaw.

% TODO:
% - electrical current profile
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

Reqr.RW = wheel_sizing();

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
% To get some details on the formulae stated here, see the report.

% Local aliases
tf = RotDesc.settlingTime;
ti = 0;  % Initial time [s]

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
function Sizing = wheel_sizing()
% WHEEL_SIZING  Propose a first sizing for the reaction wheels.

Sizing.test = 0;
end

%% 4. Requirements plot

function Reqr = plot_requirements(Reqr)
figure("WindowStyle", "docked");
title("Time evolutions that satisfy the requirements");

% Build time samples for the plots
Reqr.RollEvo.tSample  = linspace(0, Reqr.RollEvo.duration);
Reqr.PitchEvo.tSample = linspace(0, Reqr.PitchEvo.duration);
Reqr.YawEvo.tSample   = linspace(0, Reqr.YawEvo.duration);

% Torques
subplot(2, 2, 1);
hold on
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
ylabel("Momentum (kg/(m^2*s)");
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
end
