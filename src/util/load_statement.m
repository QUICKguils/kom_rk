function Stm = load_statement
% LOAD_STATEMENT  Information provided by the project statement.
%
% This function returns a structure that contains the data provided by
% the statement, which are used throughout the project.

% TODO: add performance requirements

% Millennium Falcon
Stm.Falcon.diameter = 20;     % Diameter [m]
Stm.Falcon.height   = 8;      % Height [m]
Stm.Falcon.mass     = 100e3;  % Mass (without control system) [kg]

% Requirement on the control system
Stm.Ctrl.massMax    = 10e3;   % Max. mass [kg]
Stm.Ctrl.powerMax   = 1e6;    % Max. input power [W]
Stm.Ctrl.voltageMax = 100e3;  % Max. applied voltage [V]

% Reaction wheels
Stm.RW.beta     = deg2rad(63.4);  % Elevation angle of the pyramid [rad]
Stm.RW.speedMax = 7000 * pi/30;   % Max. angular speed [rad/s]
Stm.RW.elecR    = 50;             % Electrical resistance [ohm]
Stm.RW.torqueCst = 1;             % Motor torque constant [N*m/A]
Stm.RW.damping   = 1e-4;          % Damping factor [N*m*s]
Stm.RW.density   = 8e3;           % Density of steel composing the wheels [kg/m³]

% Thrusters
Stm.Thruster.thrust = 1e3;  % Thrust [N]
% TODO: add more if we ineed use thrusters.

end
