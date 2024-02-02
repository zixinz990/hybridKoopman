close all; clear all; clc;

% Simulate to get a state trajectory
num_steps = 100;
dt = 0.01;
m = 1;
g = -10;
x0 = [0; 1]; % [height; velocity]
state_traj = zeros(2, num_steps+1);
state_traj(:, 1) = x0;
for i = 1:num_steps
    state_traj(:, i+1) = bouncing_ball_1_dim_discrete_time(state_traj(:, i), 0, m=m, g=g, dt=dt);
end

% Plot
figure(1);
subplot(2, 1, 1);
plot(0:dt*1000:num_steps*dt*1000, state_traj(1, :), '-o', 'LineWidth', 2), grid on, axis tight;
ylabel("Height (m)", 'FontSize', 16), xlabel("Times (ms)", "FontSize", 16);
subplot(2, 1, 2);
plot(0:dt*1000:num_steps*dt*1000, state_traj(2, :), '-o', 'LineWidth', 2), grid on, axis tight;
ylabel("Velocity (m/s)", 'FontSize', 16), xlabel("Times (ms)", "FontSize", 16);
