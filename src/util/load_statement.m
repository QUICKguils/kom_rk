function Stm = load_statement
% LOAD_STATEMENT  Information provided by the project statement.
%
% This function returns a structure that contains the data provided by
% the statement, which are used throughout the project.

% Plane.
Stm.Plane.Ixx = 6e3;   % Moment of inertia about the x-axis [kg*m²].
Stm.Plane.Iyy = 50e3;  % Moment of inertia about the y-axis [kg*m²].
Stm.Plane.Izz = 50e3;  % Moment of inertia about the z-axis [kg*m²].
Stm.Plane.DIM = [13.4, 11.76, 2.4]; % Dimensions [m].

% Reaction wheels.
Stm.BETA       = 63.4;         % Elevation angle of the pyramid [°].
Stm.DTHETA_MAX = 7e3 * pi/30;  % Maximum angular speed [rad/s].
Stm.R_E        = 50;           % Internal resistance of the motor [ohm].
Stm.C_T        = 1;            % Torque constant of the motor [Nm/A].
Stm.ZETA       = 1e-4;         % Damping factor of the bearings and lubricant [Nm/(rad*s)].

end
