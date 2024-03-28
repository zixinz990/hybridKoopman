close all; clear; clc;

% Setup parameters
num_steps = 100;
dt = 0.01;
m = 1;
g = 10;
x0 = [1; 0]; % [height; velocity]
dyn_fun = @bouncing_ball_2_dim_dyn;

% Initialize trajectory
x_traj = zeros(2, num_steps); % a fat matrix
x_traj(:, 1) = x0;

% Simulate
for i = 1:num_steps
    x_traj(:, i+1) = dyn_fun(x_traj(:, i), 20, m = m, g = g, dt = dt);
end

% Plot
figure(1);

subplot(2, 1, 1);
plot(0:dt*1000:num_steps*dt*1000, x_traj(1, :), '-o', 'LineWidth', 2);
grid on, axis tight;
xlabel("Times (ms)", "FontSize", 16);
ylabel("Height (m)", 'FontSize', 16);
title("Height", "FontSize", 24)

subplot(2, 1, 2);
plot(0:dt*1000:num_steps*dt*1000, x_traj(2, :), '-o', 'LineWidth', 2);
grid on, axis tight;
xlabel("Times (ms)", "FontSize", 16);
ylabel("Velocity (m/s)", 'FontSize', 16);
title("Velocity", "FontSize", 24)
