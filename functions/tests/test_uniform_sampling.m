close all; clear all; clc;

% Sample
x_range = [0, 1; ...
           -0.5, 0.5];
step_size = 0.01;
data = uniform_sampling_2_dim(x_range, step_size);

% Plot
figure(1);
plot(data(:, 1), data(:, 2), '.'), axis equal, grid on;
xlabel("Height (m)", 'FontSize', 16), ylabel("Velocity (m/s)", "FontSize", 16);
