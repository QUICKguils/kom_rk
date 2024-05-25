# Spacecraft Control Project

This repository holds all the code written for the project carried out as part
of the Spacecraft Control course (AERO0036), academic year 2023-2024.

## Basic usage

From the top level directory (where this README lies), just run the main project
file:
```matlab
run src\main.m
```
The execution of this main function add all the project files to the matlab
path, for the current matlab session. Assuming that the top level directory path
is called `rootDir`, it is still possible to add the project files manually to
the matlab path:
```matlab
addpath(genpath(rootDir));
```

## Advanced usage

### Options of the main file

The default code execution parameters are stored in the
`src/util/load_defaults.m` file.
It is however possible to launch the main function overriding these default
values.
The docstring of the main function gives an exhaustive usage explanation.
In particular, it is possible to decide whether the relevant data that were
computed should be saved. If yes, a `res/` directory will be created at top
level, and will store these data in appropriate `.mat` files.

For example:
```matlab
% Override some default code execution parameters:
RunArg.opts = 's';  % Only save the computed data, don't plot them.

% Launch the main function.
main(RunArg);
```

## Project architecture

- `src/`:
  - `main.m` triggers all the code of the project.
  - `requirements.m` provides an estimation of the spacecraft attitude and power
    needs that meets the performance requirements.
  - `ss_model.m` provides the state-space representations of the spacecraft
    dynamics in roll, pitch and yaw.
  - `lqr_control.m` provides a controller derived from a linear quadratic
    regulation (LQR).
  - `pid_control.m` provides a controller derived from a
    proportional-integral-derivative control (PID control).
  - `util/`: collection of utility functions that are used throughout the
    project.
  - `slx/`: Simulink models for LQR and PID control.
- `res/`: contains the `.mat` files that hold the relevant data that were
  computed throughout the project. This directory is automatically created, if
  absent on the user's machine.
