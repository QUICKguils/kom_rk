function requirements(RunArg, Stm)
% REQUIREMENTS  Perfomances required for the attitude control system.
%
% Arguments:
%   Stm    (struct) -- Project statement data.
%   RunArg (struct) -- Code execution parameters, with fields:
%     opts  (1xN char) -- Output options.
%       'p' -> Enable [P]lots creation.

% Unpack relevant execution parameters.
LocalRunArg = {RunArg.opts};
opts = LocalRunArg{:};

% TODO: make 100% sure of these formulae
I_xx = 0.5 * Stm.Falcon.mass * Stm.Falcon.diameter^2 / 4;
I_yy = I_xx;
I_zz = 2 * I_xx;

% See calculus details in the report
M_x = pi/50 * I_xx;

theta_dot_pre = @(t) M_x/I_xx .* t;
theta_dot_post = @(t) M_x/I_xx .* (10 - t);

theta_pre  = @(t) 0.5 * M_x/I_xx * t.^2;
theta_post = @(t) pi/2 - 0.5 * M_x/I_xx * (10-t).^2;

t_pre = 0:0.2:5;
t_post = 5:0.2:10;

figure("WindowStyle", "docked");

subplot(3, 1, 1);
hold on
plot(t_pre, theta_pre(t_pre));
plot(t_post, theta_post(t_post));
grid;
xlabel("Time (s)");
ylabel("Theta (rad)");

subplot(3, 1, 2);
hold on
plot(t_pre, theta_dot_pre(t_pre));
plot(t_post, theta_dot_post(t_post));
grid;
xlabel("Time (s)");
ylabel("Theta_dot (rad/s)");

subplot(3, 1, 3);
hold on
plot(t_pre, M_x*ones(1, numel(t_pre)));
plot(t_post, -M_x*ones(1, numel(t_post)));
grid;
xlabel("Time (s)");
ylabel("Torque (N*m)");

end