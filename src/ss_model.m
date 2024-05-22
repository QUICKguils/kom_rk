function SS = ss_model(RunArg, Stm, SS)

[Roll.A,  Roll.B,  Roll.C,  Roll.D]  = get_roll_ss(Stm);
% [Pitch.A, Pitch.B, Pitch.C, Pitch.D] = get_pitch_ss(Stm);
% [Yaw.A,   Yaw.B,   Yaw.C,   Yaw.D]   = get_yaw_ss(Stm);

SS.Roll  = Roll;
% SS.Pitch = Pitch;
% SS.Yaw   = Yaw;
end

function [A, B, C, D] = get_roll_ss(Stm)
% GET_ROLL_SS  Get the state-space representation of the roll equations.

% Local aliases
Ixx = Stm.Falcon.Ixx;
RW  = Stm.RW;

A = [0, 1;
	 0, sin(Stm.beta)/Ixx * (RW.torqueCst^2/RW.elecR + RW.damping) * (-2*sin(beta) - Ixx/(Iw*sin(beta)))];
B = [0;
	sin(beta)*RW.torqueCst/(Ixx*RW.elecR)];
C = [1 0];
D = 0;

end

function [A, B, C, D] = get_pitch_ss(Stm)
% GET_PITCH_SS  Get the state-space representation of the pitch equations.

% Local aliases
Iyy = Stm.Falcon.Iyy;
RW  = Stm.RW;

% A = ;
% B = ;
% C = ;
% D = ;

end

function [A, B, C, D] = get_yaw_ss(Stm)
% GET_YAW_SS  Get the state-space representation of the yaw equations.

% Local aliases
Izz = Stm.Falcon.Izz;
RW  = Stm.RW;

% A = ;
% B = ;
% C = ;
% D = ;

end
