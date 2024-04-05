function requirements(RunArg, Stm)
	% REQUIREMENTS  Minimum torque and angular momentum required.
	%
	% Arguments:
	%   Stm    (struct) -- Project statement data.
	%   RunArg (struct) -- Code execution parameters, with fields:
	%     sdiv  (int)      -- Number of subdivisions in the bare structure.
	%     nMode (int)      -- Number of first mode computed.
	%     opts  (1xN char) -- Output options.
	%       'p' -> Enable [P]lots creation.
	% Returns:
	%   Req (struct) --

	% Unpack relevant execution parameters.
	LocalRunArg = {RunArg.sdiv, RunArg.nMode, RunArg.opts};
	[sdiv, nMode, opts] = LocalRunArg{:};

	% Requirements data.

% Requirement 1: roll.
% X-wing shall be able to change its orientation of 90° in roll in 3 s.
	R1.time_f  = 3;     % Max. time interval for orientation [s].
	R1.theta_f = pi/4;  % Desired orientation angle [rad].
% Requirement 2: pitch.
% X-wing shall be able to change its orientation of 30° in pitch in 5 s.
	R2.time_f  = 5;     % Max. time interval for orientation [s].
	R2.theta_f = pi/6;  % Desired orientation angle [rad].
% Requirement 3: yaw.
% When its shield is hit by a laser, it creates a yaw torque of 4 kN*m
% for 0.5 s. The X- wing shall be able to recover it initial orientation
% within 5 s.
	R3.time_f = 5;    % Max. time interval for recovery [s].
	R3.time_i = 0.5;  % Duration of the laser shot [s].
	R3.T_i    = 4e3;  % Torque induced by the laser [N*m].
	R3.theta_f = pi/12;

	[T1, H1] = orientation(R1, C.Plane.Ixx);
	[T2, H2] = orientation(R2, C.Plane.Iyy);
	[T3, H3] = orientation(R3, C.Plane.Izz);

	% Find the maximum of


	% Plot and/or write data, if desired.
	if contains(opts, 'p') || contains(opts, 'w')
		plot_write();
	end

	%% Requirement: imposed orientation time.

	function [T, H] = orientation(R, I)
		% ORIENTATION  Minimum torque and momentum for the orientation.

		% Torque and momentum definition. See report for details.
		T_cst = 8*I * R.theta_f/R.time_f^2;
		T = @(t) T_cst .* (t <= R.time_f/2) - T_cst .* (t > R.time_f/2);
		H = @(t) arrayfun(@(ti) integral(T, 0, ti), t);
	end

	%% Requirement: imposed recovery time.

	function [T, H] = recovery(R, I)
		% RECOVERY  Minimum torque and momentum for the recovery.

		% Torque and momentum definition. See report for details.
		T = 0 * R.time_i; H = 0 * I;
	end

	%% Plot and write data.

	function plot_write()
		% PLOT_WRITE  Plot data and/or write them in external file.

		if contains(opts, 'p')
			% Angular momentum.
			subplot(2, 1, 1);
			hold on;
			fplot(T1, [0, R1.time_f], 'linewidth', 1);
			fplot(T2, [0, R2.time_f], 'linewidth', 1);
			fplot(T3, [0, R3.time_f], 'linewidth', 1);
			hold off;
			title('Evolution of the torque.');
			xlabel('Time (s)');
			ylabel('H(t)/(N*m)');
			grid;
			subplot(2, 1, 2);
			hold on;
			fplot(H1, [0, R1.time_f], 'linewidth', 1);
			fplot(H2, [0, R2.time_f], 'linewidth', 1);
			fplot(H3, [0, R3.time_f], 'linewidth', 1);
			hold off;
			title('Evolution of the angular momentum.');
			xlabel('Time (s)');
			ylabel('H(t)/(N*m/s)');
			grid;
		end

		if contains(opts, 'w')
			disp("writing to file...");
		end
	end

end
