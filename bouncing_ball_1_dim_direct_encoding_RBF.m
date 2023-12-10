close all; clear; clc;

syms v real

%% Dynamics
dt = 0.001;
g = -9.81;
v_lb = -0.5;
v_ub = 0.5;
f1 = v + g * dt;
f2 = v + g * dt + 1;
f_list = [f1; f2];
f_range = [v_lb - g * dt, v_ub; ...
           v_lb, v_lb - g * dt];
f1_fun = matlabFunction(f1);
f2_fun = matlabFunction(f2);

%% Simulation
v_init = 0;
sim_step = 102 * 5;
v_gt_list = zeros(sim_step+1, 1);
v_gt_list(1) = v_init;

for i = 1:sim_step
    v_gt_list(i+1) = f1_fun(v_gt_list(i));
    if v_gt_list(i+1) <= -0.5
        v_gt_list(i+1) = v_gt_list(i) + g * dt + 1;
    end
end

figure(1);
plot(0:sim_step, v_gt_list);
xlabel("Steps"), ylabel("Velocity");

%% Choose RBFs
eps = 20;
radius = -log(0.5) / eps;
n_obs = 100;
[idx, C] = kmeans(v_gt_list, n_obs);
C = sort(C);
% for k = 1:n_obs
%     viscircles([0, C(k)], radius);
% end
% axis equal;
g_list = sym(zeros(n_obs, 1));
for n = 1:n_obs
    g_list(n) = exp(-eps*abs(v-C(n)));
end

%% Calculate Q and R
Q = cal_Q_1_dim(g_list, f_list, f_range);
R = cal_R_1_dim(g_list, v_lb, v_ub);
A = Q * inv(R);

%% Predict
g_pred_list = zeros(n_obs, sim_step+1);
g_pred_list(:, 1) = double(subs(g_list, v, v_init));
v_pred_list = zeros(sim_step+1, 1);
v_pred_list(1) = v_init;
for i = 1:sim_step
    g_pred_list(:, i+1) = A * g_pred_list(:, i);
    [max_g, idx] = max(g_pred_list(:, i+1));
    g_pred_list(:, i+1) = g_pred_list(:, i+1) / max_g;
    v_pred_list(i+1) = C(idx);
end

hold on, plot(0:sim_step, v_pred_list);
legend("Ground Truth", "Prediction");