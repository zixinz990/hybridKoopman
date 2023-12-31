close all; clear; clc;

addpath("./functions/");
syms k v real

%% Dynamics
dt = 0.001;
g = -9.81;
f = v + g * dt;
f_list = f;
x_lb = -0.5;
x_ub = 0.5;
f_range = [-0.5, 0.5];

%% Observables
n_obs = 17; % must be odd
g_list = sym(zeros(n_obs, 1));
g_list(1) = sym(1);
for n = 1:(n_obs - 1) / 2
    g_list(2*n) = cos(2*pi*n*v);
    g_list(2*n+1) = sin(2*pi*n*v);
end

%% Orthonormal basis functions
phi_k = exp(2*pi*1i*k*v);
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

%% Predict
v_init = 0.0;
total_length = 500;
g_list_predict = zeros(n_obs, total_length);

g_list_predict(:, 1) = double(subs(g_list, v, v_init));
for t = 2:total_length
    g_list_predict(:, t) = A * g_list_predict(:, t-1);
end

%% Reconstruct state
v_est_list = zeros(total_length, 1);
for t = 1:total_length
    g3 = real(g_list_predict(3, t));
    g2 = real(g_list_predict(2, t));
    v_est_list(t) = atan2(g3, g2) / (2 * pi);
    if v_est_list(t) < 0
        v_est_list(t) = v_est_list(t);
    end
end

%% Ground truth
v_gt_list = zeros(total_length, 1);
v_gt_list(1) = v_init;
for k = 2:total_length
    v_gt_list(k) = subs(f, v, v_gt_list(k-1));
    if v_gt_list(k) <= -0.5
        v_gt_list(k) = 0.5;
    end
end

%% Plot
plot(1:total_length, v_gt_list, 'LineWidth', 1), hold on;
plot(1:total_length, v_est_list, 'LineWidth', 2, 'LineStyle', '-.');
legend("Ground Truth", "Prediction");
xlim([0, total_length]);
grid on;