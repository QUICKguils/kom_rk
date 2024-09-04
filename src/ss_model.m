function SS = ss_model(Stm, Reqr)
%  SS_MODEL  Get the s.s. representations of the spacecraft dynamics.
%
% The returned state-space representations are derived from the set of
% equations provided by the appendix of the project statement.
%
% Arguments:
%   Stm  (struct) -- Project statement data
%   Reqr (struct) -- Requirements estimation
% Return:
%   SS (struct) -- State-space representations, with fields:
%     Roll  (struct) -- A, B, C and D matrices for the roll dynamics
%     Pitch (struct) -- A, B, C and D matrices for the pitch dynamics
%     Yaw   (struct) -- A, B, C and D matrices for the yaw dynamics

SS.Roll  = ss_roll(Stm, Reqr);
SS.Pitch = ss_pitch(Stm, Reqr);
SS.Yaw   = ss_yaw(Stm, Reqr);
end

function SS = ss_roll(Stm, Reqr)
% SS_ROLL  Get the state-space representation of the roll equations.
%
% This function simply rearrange the roll ODE of motion in a
% state-space representation (see lesson 2).

% Local aliases
Ixx = Stm.Komrk.Ixx;
RW  = Stm.RW;
Irw = Reqr.Sizing.Irw;

% A and B: put all the dynamics in the last row
SS.A = [
	0, 1;
	0, sin(RW.beta)/Ixx * (RW.torqueCst^2/RW.elecR + RW.damping) * (-2*sin(RW.beta) - Ixx/(Irw*sin(RW.beta)))
];
SS.B = [
	0;
	sin(RW.beta)*RW.torqueCst/(Ixx*RW.elecR)
];

% C and D: output y(t) is the state vector x(t)
SS.C_MIMO = eye(2);
SS.D_MIMO = [0; 0];
% SISO version: only keep rotation angle as output
SS.C_SISO = [1, 0];
SS.D_SISO = 0;
end

function SS = ss_pitch(Stm, Reqr)
% SS_PITCH  Get the state-space representation of the pitch equations.
%
% This function simply rearrange the pitch ODE of motion in a
% state-space representation (see lesson 2).

% Local aliases
Iyy = Stm.Komrk.Iyy;
RW  = Stm.RW;
Irw = Reqr.Sizing.Irw;

% A and B: put all the dynamics in the last row
SS.A = [
	0, 1;
	0, sin(RW.beta)/Iyy * (RW.torqueCst^2/RW.elecR + RW.damping) * (-2*sin(RW.beta) - Iyy/(Irw*sin(RW.beta)))
];
SS.B = [
	0;
	sin(RW.beta)*RW.torqueCst/(Iyy*RW.elecR)
];

% C and D: output y(t) is the state vector x(t)
SS.C_MIMO = eye(2);
SS.D_MIMO = [0; 0];
% SISO version: only keep rotation angle as output
SS.C_SISO = [1, 0];
SS.D_SISO = 0;
end

function SS = ss_yaw(Stm, Reqr)
% SS_YAW  Get the state-space representation of the yaw equations.
%
% This function simply rearrange the yaw ODE of motion in a
% state-space representation (see lesson 2).

% Local aliases
Izz = Stm.Komrk.Izz;
RW  = Stm.RW;
Irw = Reqr.Sizing.Irw;

% A and B: put all the dynamics in the last row
SS.A = [
	0, 1;
	0, cos(RW.beta)/Izz * (RW.torqueCst^2/RW.elecR + RW.damping) * (-4*cos(RW.beta) - Izz/(Irw*cos(RW.beta)))
];
SS.B = [
	0;
	cos(RW.beta)*RW.torqueCst/(Izz*RW.elecR)
];

% C and D: output y(t) is the state vector x(t)
SS.C_MIMO = eye(2);
SS.D_MIMO = [0; 0];
% SISO version: only keep rotation angle as output
SS.C_SISO = [1, 0];
SS.D_SISO = 0;
end
