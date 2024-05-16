function main(RunArg)
	% MAIN  Trigger all the code of the project.
	%
	% Argument:
	%   opts (1xN char) -- Output options.
	%     'p' -> Enable [P]lots creation.
	%     's' -> [S]ave generated data.
	%
	% The default values used to run this function
	% are stored in util/load_defaults.m

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
	requirements(RunArg, Stm);

	% 2. State-space model
	% ss_model();

	%% Save generated data

	if contains(RunArg.opts, 's')
		save(fullfile(resDirectory, "runArguments.mat"), "-struct", "RunArg");
		save(fullfile(resDirectory, "statement.mat"),    "-struct", "Stm");
	end
end
