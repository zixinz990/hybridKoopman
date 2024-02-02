close all; clear; clc;

% Sample
g = -10;
h0_min = 0;
h0_max = 1;
v0_min = -0.5;
v0_max = 0.5;
x_init_density = 0.1;
x_init_range = [h0_min, h0_max; ...
                v0_min, v0_max];
num_steps = 100;
data = traj_sampling_2_dim(x_init_range, @bouncing_ball_1_dim_discrete_time, ...
                           num_steps, x_init_density=x_init_density);

% Plot
subplot(1, 3, 1);
plot(data(:, 1), data(:, 2), '.'), axis equal, grid on;
title("Data Points")

subplot(1, 3, 2);
[~, rbf_center_list] = kmeans(data, 300);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), '.'), axis equal, grid on;
title("Center of RBFs")

subplot(1, 3, 3);
state_DT = delaunayTriangulation(data);
triplot(state_DT), axis equal, grid on;
title("Triangulation")
