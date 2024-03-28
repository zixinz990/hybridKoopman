close all; clear; clc;

% Setup parameters
num_steps = 100;

mass = 1;
g = 10;
mu = rand(3, 1);
pos_switch = rand(3, 1) * 10 - 5;
dt = 0.01;

x0 = [-5; 10]; % [pos; vel]
dyn_fun = @box_var_friction_surface_dyn;

% Initialize trajectory
x_traj = zeros(2, num_steps); % a fat matrix
x_traj(:, 1) = x0;

% Simulate
for i = 1:num_steps
    x_traj(:, i+1) = dyn_fun(x_traj(:, i), 0, ...
                             mass=mass, g=g, mu=mu, ...
                             pos_switch=pos_switch, dt=dt);
end

% Plot
figure(1);

subplot(2, 1, 1);
plot(0:dt*1000:num_steps*dt*1000, x_traj(1, :), 'LineWidth', 1);
grid on, axis tight;
xlabel("Times (ms)", "FontSize", 16);
ylabel("Position (m)", 'FontSize', 16);
title("Position", "FontSize", 24)

subplot(2, 1, 2);
plot(0:dt*1000:num_steps*dt*1000, x_traj(2, :), 'LineWidth', 1);
grid on, axis tight;
xlabel("Times (ms)", "FontSize", 16);
ylabel("Velocity (m/s)", 'FontSize', 16);
title("Velocity", "FontSize", 24)
