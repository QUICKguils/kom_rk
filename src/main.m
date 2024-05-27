function main(RunArg)
% MAIN  Trigger all the code of the project.
%
% Argument:
%   opts (1xN char) -- Output options
%     'p' -> Enable [P]lots creation
%     's' -> [S]ave generated data
%  selsim ({'roll', 'pitch', 'yaw'}) -- Select simulink model
%    'roll'  -> Run the roll models
%    'pitch' -> Run the pitch models
%    'yaw'   -> Run the yaw models
%
% The default values used to run this function are stored in
% util/load_defaults.m

%% Set program initial state

% Close previous plots
close all;

% Find the root directory of the project
rootDirectory = fullfile(fileparts(mfilename('fullpath')), "..");

% Create the untracked results directory, if absent
resDirectory = fullfile(rootDirectory, "/res");
if ~isfolder(resDirectory)
	mkdir(resDirectory);
end

% Add resursively sub-directories in the Matlab path
addpath(genpath(fullfile(rootDirectory, "src")));

%% Options setting

% Fetch the defaults execution parameters
Default = load_defaults();

% Overwrite these defaults with user input
switch nargin
	case 0
		RunArg = Default;
	case 1
		for fn = fieldnames(Default)'
			if ~isfield(RunArg, fn)
				RunArg.(fn{:}) = Default.(fn{:});
			end
		end
	otherwise
		error("Wrong number of input parameters.");
end

%% Execute the code

% 0. Load the project statement data
Stm = load_statement();

% 1. Performance requirements
Reqr = requirements(RunArg, Stm);

% 2. State-space model
SS = ss_model(Stm, Reqr);

% 3. LQR control
Lqr = lqr_control(RunArg, Stm, SS);

% 4. PID control
Pid = pid_control(RunArg, Stm, SS);

%% Save generated data

if contains(RunArg.opts, 's')
	save(fullfile(resDirectory, "runArguments.mat"), "-struct", "RunArg");
	save(fullfile(resDirectory, "statement.mat"),    "-struct", "Stm");
	save(fullfile(resDirectory, "requirements.mat"), "-struct", "Reqr");
	save(fullfile(resDirectory, "stateSpace.mat"),   "-struct", "SS");
	save(fullfile(resDirectory, "lqrControl.mat"),   "-struct", "Lqr");
	save(fullfile(resDirectory, "pidControl.mat"),   "-struct", "Pid");
end
end
