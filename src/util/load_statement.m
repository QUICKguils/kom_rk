function Stm = load_statement
% LOAD_STATEMENT  Information provided by the project statement.
%
% This function returns a structure that contains the data provided by
% the statement, which are used throughout the project.


% Kom'rk spacecraft
Stm.Komrk.length = 68;    % Length along the roll axis [m]
Stm.Komrk.span   = 52;    % Length along the pitch axis [m]
Stm.Komrk.mass   = 30e3;  % Mass (without control system) [kg]
Stm.Komrk.Ixx    = 1e6;   % Inertia along the roll axis [kg*m²]
Stm.Komrk.Iyy    = 1e6;   % Inertia along the pitch axis [kg*m²]
Stm.Komrk.Izz    = 2e6;   % Inertia along the yaw axis [kg*m²]

% Requirement on the control system
Stm.Acs.maxMass    = 5e3;     % Maximum mass [kg]
Stm.Acs.maxPower   = 0.75e6;  % Maximum input power [W]
Stm.Acs.maxVoltage = 100e3;   % Maximum applied voltage [V]

% Reaction wheels
Stm.RW.beta      = deg2rad(63.4);  % Elevation angle of the pyramid [rad]
Stm.RW.maxSpeed  = 7000 * pi/30;   % Max. angular speed [rad/s]
Stm.RW.elecR     = 50;             % Electrical resistance [ohm]
Stm.RW.torqueCst = 1;              % Motor torque constant [N*m/A]
Stm.RW.damping   = 1e-4;           % Damping factor [N*m*s]
Stm.RW.density   = 8e3;            % Density of wheels material (steel) [kg/m³]

% Thrusters
% TODO: add thrusters statement data if we need to use them

% Performance requirements
Stm.Roll.angle          = deg2rad(90);  % Roll rotation angle [rad]
Stm.Roll.maxOvershoot   = 0.2;          % Maximum overshoot
Stm.Roll.settlingTime   = 5;            % Settling time [s]
Stm.Roll.settlingRtol   = 0.1;          % Settling relative tolerance
Stm.Pitch.angle         = deg2rad(30);  % Pitch rotation angle [rad]
Stm.Pitch.maxOvershoot  = 0.05;         % Maximum overshoot
Stm.Pitch.settlingTime  = 2.5;          % Settling time [s]
Stm.Pitch.settlingAngle = deg2rad(2);   % Settling error angle [rad]
Stm.Yaw.recoveryTime    = 5;            % Laser hit recovery time [s]
Stm.Yaw.laserTorque     = 4000;         % Torque induced by laser hit [N/m]
Stm.Yaw.laserTime       = 0.5;          % Duration of laser hit [s]
Stm.Yaw.settlingRtol    = 0.05;         % Settling relative tolerance

end
