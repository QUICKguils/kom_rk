% REFRESH_WORKSPACE  Bring computed data in global workspace.

% Find the results directory
resDirectory = fullfile(fileparts(mfilename('fullpath')), "../../res/");

Stm    = load(fullfile(resDirectory, "statement.mat"));
RunArg = load(fullfile(resDirectory, "runArguments.mat"));
Reqr   = load(fullfile(resDirectory, "requirements.mat"));
SS     = load(fullfile(resDirectory, "stateSpace.mat"));
Lqr    = load(fullfile(resDirectory, "lqrControl.mat"));
Pid    = load(fullfile(resDirectory, "pidControl.mat"));

clear resDirectory;
