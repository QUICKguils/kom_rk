function SS = ss_model(Stm)
%  SS_MODEL  Get the s.s. representations of the spacecraft dynamics.
%
% The returned state-space representations are derived from the appendix
% of the project statement.
%
% Arguments:
%   Stm    (struct) -- Project statement data
% Return:
%   SS (struct) -- State-space representation, with fields:
%     Roll  (struct) -- A, B, C and D matrices for the roll dynamics.
%     Pitch (struct) -- A, B, C and D matrices for the pitch dynamics.
%     Yaw   (struct) -- A, B, C and D matrices for the yaw dynamics.

[Roll.A,  Roll.B,  Roll.C,  Roll.D]  = ss_roll(Stm);
[Pitch.A, Pitch.B, Pitch.C, Pitch.D] = ss_pitch(Stm);
[Yaw.A,   Yaw.B,   Yaw.C,   Yaw.D]   = ss_yaw(Stm);

SS.Roll  = Roll;
SS.Pitch = Pitch;
SS.Yaw   = Yaw;
end

function [A, B, C, D] = ss_roll(Stm)
% SS_ROLL  Get the state-space representation of the roll equations.
%
% This function simply rearrange the roll ODE of motion in a
% state-space representation (see lesson 2).

% Local aliases
Ixx  = Stm.Falcon.Ixx;
RW   = Stm.RW;

Iw = 1; % TODO: remove that when implemented in requirements.m

% A and B: put all the dynamics in the last row
A = [
	0, 1;
	0, sin(RW.beta)/Ixx * (RW.torqueCst^2/RW.elecR + RW.damping) * (-2*sin(RW.beta) - Ixx/(Iw*sin(RW.beta)))
];
B = [
	0;
	sin(RW.beta)*RW.torqueCst/(Ixx*RW.elecR)
];

% C and D: output y(t) is the state vector x(t)
C = [1 0];
D = 0;
end

function [A, B, C, D] = ss_pitch(Stm)
% SS_PITCH  Get the state-space representation of the pitch equations.
%
% This function simply rearrange the pitch ODE of motion in a
% state-space representation (see lesson 2).

% Local aliases
Iyy = Stm.Falcon.Iyy;
RW  = Stm.RW;

Iw = 1; % TODO: remove that when implemented in requirements.m

% A and B: put all the dynamics in the last row
A = [
	0, 1;
	0, sin(RW.beta)/Iyy * (RW.torqueCst^2/RW.elecR + RW.damping) * (-2*sin(RW.beta) - Iyy/(Iw*sin(RW.beta)))
];
B = [
	0;
	sin(RW.beta)*RW.torqueCst/(Iyy*RW.elecR) 
];

% C and D: output y(t) is the state vector x(t)
C = [1 0];
D = 0;
end

function [A, B, C, D] = ss_yaw(Stm)
% SS_YAW  Get the state-space representation of the yaw equations.
%
% This function simply rearrange the yaw ODE of motion in a
% state-space representation (see lesson 2).

% Local aliases
Izz = Stm.Falcon.Izz;
RW  = Stm.RW;

Iw = 1; % TODO: remove that when implemented in requirements.m

% A and B: put all the dynamics in the last row
A = [
	0, 1;
	0, cos(RW.beta)/Izz * (RW.torqueCst^2/RW.elecR + RW.damping) * (-4*cos(RW.beta) - Izz/(Iw*cos(RW.beta)))
];
B = [
	0;
	cos(RW.beta)*RW.torqueCst/(Izz*RW.elecR) 
];

% C and D: output y(t) is the state vector x(t)
C = [1 0];
D = 0;
end
