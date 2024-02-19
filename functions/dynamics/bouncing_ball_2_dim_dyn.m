function x1 = bouncing_ball_2_dim_dyn(x0, u0, options)
% The discrete dynamics of a bouncing ball with elastic collision
% with flat ground
% state x = [height; velocity] = [h; v]
% control u = force (only consider z direction)
% Input:
%   x0:         initial state, 2x1 col vector
%   u0:         control input of this step
%   options.dt: step length (sec)
%   options.m:  mass (kg)
%   options.g:  gravity constant (m/s^2), always positive
%   options.e:  coefficient of restitution (when dt is large, the
%               simulation may be wrong for small e)
% Output:
%   x1: next state after dt, 2x1 col vector
arguments
    x0(2, 1) {mustBeNumeric}
    u0(1, 1) {mustBeNumeric}
    options.dt(1, 1) {mustBeNumeric, mustBePositive} = 0.01
    options.m(1, 1) {mustBeNumeric, mustBePositive} = 1.0
    options.g(1, 1) {mustBeNumeric, mustBePositive} = 10.0
    options.e(1, 1) {mustBeNumeric, mustBePositive} = 1.0
end

% Setup odefun
acc = (u0 - options.m * options.g) / options.m;
    function dxdt = odefun(t, x)
        dxdt = [x(2); acc];
    end

% Initialize parameters
t0 = 0;
tf = options.dt;
tout = t0;
xout = x0';

% Setup ode23 options
refine = 4;
odeopt = odeset('Events', @bounce, 'OutputSel', 1, 'Refine', refine);

% Solve ODE
while t0 < tf
    [t_ode, x_ode] = ode23(@odefun, [t0, tf], x0, odeopt);
    tout = [tout; t_ode(2:end)];
    xout = [xout; x_ode(2:end, :)];
    x0(1) = 0;
    x0(2) = -options.e * x_ode(end, 2);
    odeopt = odeset(odeopt, 'InitialStep', t_ode(end)-t_ode(end-refine), ...
                    'MaxStep', t_ode(end)-t_ode(1));
    t0 = t_ode(end);
end

if xout(end, 1) <= 1e-6 && xout(end, 2) <= 0
    xout(end, 1) = 0;
    xout(end, 2) = -xout(end, 2);
end

x1 = xout(end, :)';
end

% Hybrid event function
function [value, isterminal, direction] = bounce(t, x)
value = x(1);
isterminal = 1;
direction = -1;
end
