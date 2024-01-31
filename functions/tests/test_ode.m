function test_ode()
a = -10;
    function dydt = f(t, y)
        dydt = [y(2); -a];
    end
tstart = 0;
tfinal = 0.01;
refine = 1;
y0 = [0; 0.012];
options = odeset('Events', @events, 'OutputSel', 1, 'Refine', refine);

tout = tstart;
yout = y0.';
teout = [];
yeout = [];
ieout = [];

while tstart < tfinal
    [t, y, te, ye, ie] = ode23(@f, [tstart, tfinal], y0, options);

    % Accumulate output.
    nt = length(t);
    tout = [tout; t(2:nt)];
    yout = [yout; y(2:nt, :)];
    teout = [teout; te]; % Events at tstart are never reported.
    yeout = [yeout; ye];
    ieout = [ieout; ie];

    y0(1) = 0;
    y0(2) = -1 * y(nt, 2);

    options = odeset(options, 'InitialStep', t(nt)-t(nt-refine), 'MaxStep', t(nt)-t(1));
    tstart = t(nt);
end

if yout(end, 1) <= 1e-6 && yout(end, 2) < 0
    yout(end, 2) = -yout(end, 2);
end

subplot(2, 1, 1)
plot(tout, yout(:, 1), '-o'), hold on, grid on;
subplot(2, 1, 2)
plot(tout, yout(:, 2), '-o'), hold on, grid on;
xlabel('time');
ylabel('height');
title('Ball trajectory and the events');
end
% --------------------------------------------------------------


% --------------------------------------------------------------
function [value, isterminal, direction] = events(t, y)
% Locate the time when height passes through zero in a
% decreasing direction and stop integration.
value = y(1); % Detect height = 0
isterminal = 1; % Stop the integration
direction = -1; % Negative direction only
end