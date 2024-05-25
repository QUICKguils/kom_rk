function Default = load_defaults
% LOAD_DEFAULTS  Default execution parameters used throughout the source code.

% Output options
%   'p' -> Enable [P]lots creation
%   's' -> [S]ave generated data
Default.opts = 'ps';

% Select the LQR and PID simulink model to run
%   'roll'  -> Run the roll models
%   'pitch' -> Run the pitch models
%   'yaw'   -> Run the yaw models
Default.selsim = 'roll';

end
