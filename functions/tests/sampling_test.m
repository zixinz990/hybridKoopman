close all; clear; clc;

%% TEST UNIFORM SAMPLING
% Sample
x_range = [0, 1; ...
           -0.5, 0.5];
step_size = [0.01; 0.01];
data = uniform_sampling_2_dim(x_range, step_size);

% Plot
figure(1);
plot(data(:, 1), data(:, 2), '.'), axis equal, grid on;
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling", "FontSize", 24)
axis equal; axis tight;

%% TEST TRAJECTORY SAMPLING
% Sample
x_init_range = [0, 1; ...
                -0.5, 0.5];
u_range = [-10, 10];
dyn_fun = @bouncing_ball_2_dim_dyn;
num_steps = 100;
step_size = [0.1; 0.1];
data = traj_sampling_2_dim(x_init_range, u_range, dyn_fun, num_steps, step_size=step_size);

% Plot
figure(2);
for i = 1:size(data, 2)
    traj = data{i}; % (num_steps+1)x2 tall matrix
    plot(traj(:, 1), traj(:, 2), '.'), hold on;
end
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Sampled Trajectories", "FontSize", 24)
axis equal; axis tight;

%% TEST VARIATIONAL DENSITY SAMPLING FOR 2-DIM BOUNCING BALL STATE SPACE
% Sample
x_range = [0, 1; ...
           -0.5, 0.5];
dyn_fun = @bouncing_ball_2_dim_dyn;
min_step_size = [0.005; 0.005];
max_step_size = [0.02; 0.02];
data = var_density_sampling_bouncing_ball_2_dim(x_range, min_step_size, max_step_size);

% Plot
figure(3);
plot(data(:, 1), data(:, 2), '.'), axis equal, grid on;
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Variational Density Sampling", "FontSize", 24)
axis equal; axis tight;
