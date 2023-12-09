close all; clear; clc;

addpath("functions");
syms k x real;

%% Countinuous-time Dynamics
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
n_obs = 7; % must be odd
g_list = sym(zeros(n_obs, 1));
g_list(1) = sym(1);
for n = 1:(n_obs - 1) / 2
    g_list(2*n) = cos(2*pi*n*x);
    g_list(2*n+1) = sin(2*pi*n*x);
end

%% Orthonormal basis functions
phi_k = exp(2*pi*1i*k*x);
phi_list = sym(zeros(n_obs, 1));
phi_list(1) = subs(phi_k, k, 0);
for n = 1:n_obs / 2
    phi_list(2*n) = subs(phi_k, k, n);
    phi_list(2*n+1) = subs(phi_k, k, -n);
end

%% Calculate A
As = cal_As_1_dim(phi_list, f_list, f_range);
C = cal_C_1_dim(phi_list, g_list, x_lb, x_ub);
A = C * As / C;

%% Estimate
x_list = 0:0.01:1;
total_length = length(x_list);
g_dx_est_list = zeros(n_obs, total_length);
dx_gt_list = zeros(total_length, 1);
for i = 1:total_length
    g_dx_est_list(:, i) = A * subs(g_list, x, x_list(i));
    if x_list(i) <= x_star
        dx_gt_list(i) = double(subs(f1, x, x_list(i)));
    else
        dx_gt_list(i) = double(subs(f2, x, x_list(i)));
    end
end

%% Reconstruct state
dx_est_list = zeros(total_length, 1);

% Use atan2
for t = 1:total_length
    g3 = real(g_dx_est_list(3, t));
    g2 = real(g_dx_est_list(2, t));
    dx_est_list(t) = real(atan2(g3, g2) / (2 * pi));
    if dx_est_list(t) < 0
        dx_est_list(t) = dx_est_list(t) + 1;
    end
end

%% Plot
plot(1:total_length, dx_gt_list, 'LineWidth', 1), hold on;
plot(1:total_length, dx_est_list, 'LineWidth', 2, 'LineStyle', '-.');
legend("Ground Truth", "Prediction");
xlim([0, total_length]), ylim([x_lb, x_ub]);
grid on;