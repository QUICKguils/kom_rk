function varargout = requirements(RunArg, Stm)
% REQUIREMENTS  Requirements estimation for the attitude control system.
%
% Arguments:
%   Stm    (struct) -- Project statement data
%   RunArg (struct) -- Code execution parameters, with fields:
%     opts  (1xN char) -- Output options
%       'p' -> Enable [P]lots creation

% Unpack relevant execution parameters
LocalRunArg = {RunArg.opts};
opts = LocalRunArg{:};

% 1. Get the inertias of the spacecraft

I = compute_inertia(Stm.Falcon);

% 2. Time evolution of the angles, angular momentums and torques.

Reqr.RollEvo  = evolution_from_rest(Stm.Roll, I.xx);
Reqr.PitchEvo = evolution_from_rest(Stm.Pitch, I.yy);
Reqr.YawEvo   = evolution_for_yaw(Stm.Yaw, I.zz);

% 3. Electrical current and voltage estimation

% 4. Estimation of the reaction wheels sizing

% 5. Requirements plot

if contains(opts, 'p')
	plot_requirements(Reqr);
end

% 6. Return the relevant calculated data

optrets = {I, Reqr};
varargout(1:nargout) = optrets(1:nargout);
end

%% 1. Spacecraft inertias

function I = compute_inertia(Spacecraft)
% COMPUTE_INERTIA  Compute the inertia of a spacecraft.
%
% The computed inertias are appended to the Spacecraft object.
%
% This function assumes that the Spacecraft object is analogous to a
% uniform cylinder.

I.zz = 1/2  * Spacecraft.mass *    Spacecraft.radius^2;
I.xx = 1/12 * Spacecraft.mass * (3*Spacecraft.radius^2 + Spacecraft.height^2);
I.yy = I.xx;
end

%% 2. Evolution laws

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

%% 3. Requirements plot

function Reqr = plot_requirements(Reqr)
figure("WindowStyle", "docked");
title("Time evolutions that satisfy the requirements");

% Build time samples for the plots
Reqr.RollEvo.tSample  = linspace(0, Reqr.RollEvo.duration);
Reqr.PitchEvo.tSample = linspace(0, Reqr.PitchEvo.duration);
Reqr.YawEvo.tSample   = linspace(0, Reqr.YawEvo.duration);

% Torques
subplot(3, 1, 1);
hold on
plot(Reqr.RollEvo.tSample,  Reqr.RollEvo.torque(Reqr.RollEvo.tSample));
plot(Reqr.PitchEvo.tSample, Reqr.PitchEvo.torque(Reqr.PitchEvo.tSample));
plot(Reqr.YawEvo.tSample,   Reqr.YawEvo.torque(Reqr.YawEvo.tSample));
grid;
xlabel("Time (s)");
ylabel("Torque (N*m)");
% legend('Roll', 'Pitch', 'Yaw');

% Angular moments
subplot(3, 1, 2);
hold on
plot(Reqr.RollEvo.tSample,  Reqr.RollEvo.momentum(Reqr.RollEvo.tSample));
plot(Reqr.PitchEvo.tSample, Reqr.PitchEvo.momentum(Reqr.PitchEvo.tSample));
plot(Reqr.YawEvo.tSample,   Reqr.YawEvo.momentum(Reqr.YawEvo.tSample));
grid;
xlabel("Time (s)");
ylabel("Angular momentum (kg/(m^2*s)");
% legend('Roll', 'Pitch', 'Yaw');

% Rotation angles
subplot(3, 1, 3);
hold on
plot(Reqr.RollEvo.tSample,  rad2deg(Reqr.RollEvo.angle(Reqr.RollEvo.tSample)));
plot(Reqr.PitchEvo.tSample, rad2deg(Reqr.PitchEvo.angle(Reqr.PitchEvo.tSample)));
plot(Reqr.YawEvo.tSample,   rad2deg(Reqr.YawEvo.angle(Reqr.YawEvo.tSample)));
grid;
xlabel("Time (s)");
ylabel("Rotation angle (deg)");
% legend('Roll', 'Pitch', 'Yaw');
end
