function Default = load_defaults
% LOAD_DEFAULTS  Default execution parameters used throughout the source code.

% Output options
%   'p' -> Enable [P]lots creation
%   's' -> [S]ave generated data
Default.opts = 'ps';

% Select the LQR and PID simulink models to run
%   'roll'  -> Select the roll models
%   'pitch' -> Select the pitch models
%   'yaw'   -> Select the yaw models
Default.selsim = 'roll';

% Run the LQR and PID simulink models
%   true  -> Run the selected simulink models.
%   false -> Do not activate simulink.
Default.runsim = true;

end
