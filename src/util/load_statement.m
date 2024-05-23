function Stm = load_statement
% LOAD_STATEMENT  Information provided by the project statement.
%
% This function returns a structure that contains the data provided by
% the statement, which are used throughout the project.

% Millennium Falcon
Stm.Falcon.diameter = 20;     % Diameter [m]
Stm.Falcon.radius   = 10;     % Radius [m]
Stm.Falcon.height   = 8;      % Height [m]
Stm.Falcon.mass     = 100e3;  % Mass (without control system) [kg]
[Ixx, Iyy, Izz]     = spacecraft_inertia(Stm.Falcon);  % See: src/utils/spacecraft_inertia.m
Stm.Falcon.Ixx      = Ixx;    % Inertia along the roll  axis [kg*m²]
Stm.Falcon.Iyy      = Iyy;    % Inertia along the pitch axis [kg*m²]
Stm.Falcon.Izz      = Izz;    % Inertia along the yaw   axis [kg*m²]

% Requirement on the control system
Stm.Acs.maxMass    = 10e3;   % Max. mass [kg]
Stm.Acs.maxPower   = 1e6;    % Max. input power [W]
Stm.Acs.maxVoltage = 100e3;  % Max. applied voltage [V]

% Reaction wheels
Stm.RW.beta      = deg2rad(63.4);  % Elevation angle of the pyramid [rad]
Stm.RW.maxSpeed  = 7000 * pi/30;   % Max. angular speed [rad/s]
Stm.RW.elecR     = 50;             % Electrical resistance [ohm]
Stm.RW.torqueCst = 1;              % Motor torque constant [N*m/A]
Stm.RW.damping   = 1e-4;           % Damping factor [N*m*s]
Stm.RW.density   = 8e3;            % Density of steel composing the wheels [kg/m³]

% Thrusters
% TODO: add thrusters statement data if we need to use them

% Performance requirements
Stm.Roll.angle                = deg2rad(90);  % Roll rotation angle [rad]
Stm.Roll.percentageOvershoot  = 20;           % Percentage overshoot
Stm.Roll.settlingTime         = 10;           % Settling time [s]
Stm.Roll.settlingRtol         = 0.1;          % Settling relative tolerance
Stm.Pitch.angle               = deg2rad(30);  % Pitch rotation angle [rad]
Stm.Pitch.percentageOvershoot = 5;            % Percentage overshoot
Stm.Pitch.settlingTime        = 5;            % Settling time [s]
Stm.Yaw.recoveryTime          = 5;            % Laser hit recovery time [s]
Stm.Yaw.laserTorque           = 4000;         % Torque induced by laser hit [N/m]
Stm.Yaw.laserTime             = 0.5;          % Duration of laser hit [s]
end
