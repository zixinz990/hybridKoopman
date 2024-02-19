close all; clear; clc;

%% UNIFORM SAMPLING + RBFs
% Sample
x_range = [0, 1; ...
           -0.5, 0.5];
step_size = [0.01; 0.01];
data = uniform_sampling_2_dim(x_range, step_size);

% Generate observables
num_obs = 100;
[~, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_obs, data);

% Plot
figure(1);

subplot(2, 1, 1);
plot(data(:, 1), data(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling", "FontSize", 24)
axis equal; axis tight; grid on;

subplot(2, 1, 2);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; axis tight; grid on;

%% TRAJECTORY SAMPLING + RBFs
% Sample
x_init_range = [0, 1; ...
                -0.5, 0.5];
u_range = [-10, 10];
dyn_fun = @bouncing_ball_2_dim_dyn;
num_steps = 100;
step_size = [0.1; 0.1];
data = traj_sampling_2_dim(x_init_range, u_range, dyn_fun, num_steps, step_size=step_size);
data = cell2mat(reshape(data, size(data, 2), 1));

% Generate observables
num_obs = 100;
[g, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_obs, data);

% Plot
figure(2);

subplot(2, 1, 1);
plot(data(:, 1), data(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Sampled Trajectories", "FontSize", 24)
axis equal; axis tight; grid on;

subplot(2, 1, 2);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; axis tight; grid on;

%% VARIATIONAL DENSITY SAMPLING + RBFs
% Sample
x_range = [0, 1.0125; ...
           -4.5, 4.5];
min_step_size = [0.005; 0.005];
max_step_size = [0.02; 0.02];
data = var_density_sampling_bouncing_ball_2_dim(x_range, min_step_size, max_step_size);

% Generate observables
num_obs = 500;
[g, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_obs, data);

% Plot
figure(3);

subplot(2, 1, 1);
plot(data(:, 1), data(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Variational Density Sampling", "FontSize", 24)
axis equal; axis tight; grid on;

subplot(2, 1, 2);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; axis tight; grid on;
