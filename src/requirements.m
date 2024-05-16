function varargout = requirements(RunArg, Stm)
% REQUIREMENTS  Perfomances required for the attitude control system.
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

Reqr.Roll  = rotation_requirements(Stm.Perf.Roll, I.xx);
Reqr.Pitch = rotation_requirements(Stm.Perf.Pitch, I.yy);
% Reqr.Yaw   = rotation_requirements(Stm.Perf.Yaw, I.zz);

% 3. Requirements plot

if contains(opts, 'p')
	plot_requirements(Reqr);
end

% 4. Electrical current and voltage estimation

% 5. Return the relevant calculated data

optrets = {Reqr};
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

function Rot = rotation_requirements(PerfRot, I)
% ROTATION_REQUIREMENTS  Time evolutions that satisfy the requirements.
%
% This function gives the time expressions for the torque, angular
% momentum and rotation angle so that the rotation requirement is
% satisfied.
%
% To get some details on the formulae stated here, see the report.

% Local Aliases
ts = PerfRot.settlingTime;

% Constant moment, that yield half the rotation angle
% at half the rotation time
Mc = PerfRot.angle/(ts/2)^2 * I;

% Time expressions of the torque, angular momentum and roll angle
M     = @(t) Mc            .* (t<=ts/2) - Mc                              .* (t>ts/2);
H     = @(t) Mc*t          .* (t<=ts/2) + Mc*(ts-t)                       .* (t>ts/2);
angle = @(t) Mc/(2*I)*t.^2 .* (t<=ts/2) + Mc/(2*I)*(-t.^2+2*ts*t-ts.^2/2) .* (t>ts/2);

% Build the return data structure
Rot.duration = PerfRot.settlingTime;
Rot.torque   = M;
Rot.momentum = H;
Rot.angle    = angle;
end

%% 3. Requirements plot

function Reqr = plot_requirements(Reqr)
figure("WindowStyle", "docked");
title("Time evolutions that satisfy the requirements");

% Build time samples for the plots
Reqr.Roll.tSample  = linspace(0, Reqr.Roll.duration);
Reqr.Pitch.tSample = linspace(0, Reqr.Pitch.duration);
% Reqr.Yaw.tSample   = linspace(0, Reqr.Yaw.duration);

% Torques
subplot(3, 1, 1);
hold on
plot(Reqr.Roll.tSample, Reqr.Roll.torque(Reqr.Roll.tSample));
plot(Reqr.Pitch.tSample, Reqr.Pitch.torque(Reqr.Pitch.tSample));
grid;
xlabel("Time (s)");
ylabel("Torque (N*m)");
% legend('Roll', 'Pitch', 'Yaw');

% Angular moments
subplot(3, 1, 2);
hold on
plot(Reqr.Roll.tSample, Reqr.Roll.momentum(Reqr.Roll.tSample));
plot(Reqr.Pitch.tSample, Reqr.Pitch.momentum(Reqr.Pitch.tSample));
grid;
xlabel("Time (s)");
ylabel("Angular momentum (kg/(m^2*s)");
% legend('Roll', 'Pitch', 'Yaw');

% Rotation angles
subplot(3, 1, 3);
hold on
plot(Reqr.Roll.tSample, rad2deg(Reqr.Roll.angle(Reqr.Roll.tSample)));
plot(Reqr.Pitch.tSample, rad2deg(Reqr.Pitch.angle(Reqr.Pitch.tSample)));
grid;
xlabel("Time (s)");
ylabel("Rotation angle (deg)");
% legend('Roll', 'Pitch', 'Yaw');
end
