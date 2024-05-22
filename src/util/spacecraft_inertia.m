function [Ixx, Iyy, Izz] = spacecraft_inertia(Spacecraft)
% SPACECRAFT_INERTIA  Compute the inertia of a spacecraft.
%
% This function assumes that the Spacecraft object is analogous to a
% uniform cylinder.
% The Spacecraft object is assumed to have a `mass`, `radius` and
% `height` field.

Izz = 1/2  * Spacecraft.mass *    Spacecraft.radius^2;
Ixx = 1/12 * Spacecraft.mass * (3*Spacecraft.radius^2 + Spacecraft.height^2);
Iyy = Ixx;
end