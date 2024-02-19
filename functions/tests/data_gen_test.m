% Test data generation and RBFs generation
close all; clear; clc;

% Set parameters
dyn_fun = @bouncing_ball_2_dim_dyn;
uniform_step_size = [0.01; 0.01];
traj_step_size = [0.05; 0.05];
var_density_min_step_size = [0.005; 0.005];
var_density_max_step_size = [0.02; 0.02];

%% UNIFORM SAMPLING
% Generate data
[data_uniform, data_uniform_next] = gen_data(dyn_fun, "uniform", uniform_step_size=uniform_step_size);

% Generate RBFs
num_obs = 500;
[~, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_obs, data_uniform);

% Plot
figure(1);
subplot(1, 3, 1);
plot(data_uniform(:, 1), data_uniform(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling", "FontSize", 24)
axis equal, grid on;
subplot(1, 3, 2);
plot(data_uniform_next(:, 1), data_uniform_next(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling (next)", "FontSize", 24)
axis equal, grid on;
subplot(1, 3, 3);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; grid on;

%% TRAJECTORY SAMPLING
% Generate data
[data_traj, data_traj_next] = gen_data(dyn_fun, "trajectory", traj_step_size=traj_step_size);

% Generate RBFs
num_obs = 500;
[~, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_obs, data_traj);

% Plot
figure(2);
subplot(1, 3, 1);
plot(data_traj(:, 1), data_traj(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Sampled Trajectories", "FontSize", 24)
axis equal; grid on;
subplot(1, 3, 2);
plot(data_traj_next(:, 1), data_traj_next(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Sampled Trajectories (next)", "FontSize", 24)
axis equal; grid on;
subplot(1, 3, 3);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; grid on;

%% VARIATIONAL DENSITY SAMPLING
% Generate data
[data_var_density, data_var_density_next] = gen_data(dyn_fun, "var_density", var_density_min_step_size=var_density_min_step_size, var_density_max_step_size=var_density_max_step_size);

% Generate RBFs
num_obs = 500;
[~, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_obs, 10, data_var_density);

% Plot
figure(3);
subplot(1, 3, 1);
plot(data_var_density(:, 1), data_var_density(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Variational Density Sampling", "FontSize", 24)
axis equal; grid on;
subplot(1, 3, 2);
plot(data_var_density_next(:, 1), data_var_density_next(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Variational Density Sampling (next)", "FontSize", 24)
axis equal; grid on;
subplot(1, 3, 3);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; grid on;
