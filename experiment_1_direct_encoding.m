close all; clear; clc;

addpath("functions");
syms k x real;

%% Dynamics
a = 0.31;
b = 0.94;
c = -3;
x_star = 0.32;

f1 = c * x - c * x_star;
f2 = b * x + a - b * x_star;
f_list = [f1; f2];
x_lb = 0;
x_ub = 1;
f_range = [0, x_star; x_star, 1];

%% Observables
n_obs = 33; % must be odd
g_list = sym(zeros(n_obs, 1));
g_list(1) = sym(1);
for n = 1:(n_obs - 1) / 2
    g_list(2*n) = cos(2*pi*n*x);
    g_list(2*n+1) = sin(2*pi*n*x);
end

%% Calculate Q and R
Q = cal_Q_1_dim(g_list, f_list, f_range);
R = cal_R_1_dim(g_list, x_lb, x_ub);
A = Q * inv(R);

%% Predict
x_init = 0.0;
total_length = 100;
g_list_predict = zeros(n_obs, total_length);

g_list_predict(:, 1) = double(subs(g_list, x, x_init));
for t = 2:total_length
    g_list_predict(:, t) = A * g_list_predict(:, t-1);
end

%% Reconstruct state
x_est_list = zeros(total_length, 1);

% Use atan2
for t = 1:total_length
    g3 = real(g_list_predict(3, t));
    g2 = real(g_list_predict(2, t));
    x_est_list(t) = real(atan2(g3, g2) / (2 * pi));
    if x_est_list(t) < 0
        x_est_list(t) = x_est_list(t) + 1;
    end
end

%% Ground truth
x_gt_list = zeros(total_length, 1);
x_gt_list(1) = x_init;

for t = 2:total_length
    if x_gt_list(t-1) <= x_star
        x_gt_list(t) = subs(f_list(1), x, x_gt_list(t-1));
    else
        x_gt_list(t) = subs(f_list(2), x, x_gt_list(t-1));
    end
end

%% Plot
plot(1:total_length, x_gt_list, 'LineWidth', 1), hold on;
plot(1:total_length, x_est_list, 'LineWidth', 2, 'LineStyle', '-.');
legend("Ground Truth", "Prediction");
xlim([0, total_length]), ylim([x_lb, x_ub]);
grid on;