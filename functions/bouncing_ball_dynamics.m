function x_next = bouncing_ball_dynamics(x0, u0, options)
% This function does one dynamics rollout for a 1D bouncing ball
% state = [height; velocity]
% control = [force]
% Input:
%   x0: initial state of the current step
%   u0: control of the current step
%   dt: step length
%   m: optional, mass (kg)
%   g: optional, gravity constant (m/s^2)
% Output:
%   x_next: state after dt
arguments
    x0 (2, 1) {mustBeNumeric}
    u0 (1, 1) {mustBeNumeric}
    options.dt (1, 1) {mustBeNumeric, mustBePositive} = 0.01
    options.m (1, 1) {mustBeNumeric, mustBePositive} = 1
    options.g (1, 1) {mustBeNumeric} = -10
end

a = (options.m * options.g + u0) / options.m;
    function dxdt = odefun(t, x)
        dxdt = [x(2); a];
    end

t0 = 0;
tf = options.dt;
refine = 1;
odeopt = odeset('Events', @events, 'OutputSel', 1, 'Refine', refine);

tout = t0;
xout = x0.';

while t0 < tf
    [t, x] = ode23(@odefun, [t0, tf], x0, odeopt);
    tout = [tout; t(2:end)];
    xout = [xout; x(2:end, :)];
    x0(1) = 0;
    x0(2) = -1 * x(end, 2);
    odeopt = odeset(odeopt, 'InitialStep', t(end)-t(end-refine), 'MaxStep', t(end)-t(1));
    t0 = t(end);
end

if xout(end, 1) <= 1e-6 && xout(end, 2) < 0
    xout(end, 2) = -xout(end, 2);
end

x_next = xout(end, :)';
end

function [value, isterminal, direction] = events(t, x)
value = x(1);
isterminal = 1;
direction = -1;
end
