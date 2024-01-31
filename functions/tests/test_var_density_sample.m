close all; clear; clc;

g = -10;
h0_min = 0;
h0_max = 1;
v0_abs_max = 1;

v_min = -sqrt(v0_abs_max^2 + 2 * (-g) * h0_max);
v_max = v0_abs_max;
h_min = 0;
h_max = 0.5 * v0_abs_max^2 /  (-g) + h0_max;

x_range = [h_min, h_max; v_min, v_max]
min_step_len = 0.001;
max_step_len = 0.02;
data = var_density_sample_bouncing_ball_2_dim(x_range, min_step_len, max_step_len);
plot(data(:, 1), data(:, 2), 'o'), axis equal, grid on;