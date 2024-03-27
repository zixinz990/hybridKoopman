close all; clear;
addpath("./data");
addpath("./functions");

has_state_in_obs = true;
file_name = "./data/2024_3_27_12_29_bouncing_ball_2_dim_KDE.mat";

syms h v real;
x = [h; v];

% load mat file
load(file_name);

%% Test over points
% calculate ground truth and prediction
[H0, V0] = meshgrid(0:0.01:0.5125, -3.2016:0.01:3.2016);
x0_list = [H0(:), V0(:)]; % mesh grid to points list, n_points x 2 matrix

x_next_gt_list = zeros(size(x0_list));
x_next_pred_list = zeros(size(x0_list));

for i = 1:size(x0_list, 1)
    % initial state
    x0 = x0_list(i, :)';
    g0 = g_list_fun(x0, obs_fun_cell(1:end-1)); % obs_fun_cell = [RBFs; x; u]
    
    % ground truth
    x_next_gt = bouncing_ball_2_dim_dyn(x0, 0);
    x_next_gt_list(i, :) = x_next_gt;

    % prediction
    g_next_pred = A * g0;
    if has_state_in_obs
        x_next_pred = g_next_pred(end-1:end);
    else
        % todo
    end
    x_next_pred_list(i, :) = x_next_pred';
end

pred_err = x_next_pred_list - x_next_gt_list;
h_err = reshape(abs(pred_err(:, 1)), size(H0));
v_err = reshape(abs(pred_err(:, 2)), size(H0));
fprintf("n_obs: %d, eps: %d\n", size(obs_fun_cell, 1), eps);
fprintf("Average h_err: %d. Min h_err: %d. Max h_err: %d\n", mean(h_err(:)), min(h_err(:)), max(h_err(:)))
fprintf("Average v_err: %d. Min v_err: %d. Max v_err: %d\n\n", mean(v_err(:)), min(v_err(:)), max(v_err(:)))

% plot
figure(1);

subplot(1, 2, 1);
surf(H0, V0, h_err), axis equal, view(2);
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Height Prediction (m)");

subplot(1, 2, 2);
surf(H0, V0, v_err), axis equal, view(2);
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Velocity Prediction (m/s)");

%% Test over points, but with control
% Seperate A matrix
A_new = A(1:end-1, 1:end-1);
B_new = A(1:end-1, end);

%%
% calculate ground truth and prediction
[H0, V0] = meshgrid(0:0.01:0.5125, -3.2016:0.01:3.2016);
x0_list = [H0(:), V0(:)]; % mesh grid to points list, n_points x 2 matrix
u0 = 5;

x_next_gt_list = zeros(size(x0_list));
x_next_pred_list = zeros(size(x0_list));

for i = 1:size(x0_list, 1)
    % initial state
    x0 = x0_list(i, :)';
    g0 = g_list_fun(x0, obs_fun_cell(1:end-1)); % [rbf; x]
    
    % ground truth
    x_next_gt = bouncing_ball_2_dim_dyn(x0, u0);
    x_next_gt_list(i, :) = x_next_gt;

    % prediction
    g_next_pred = A_new * g0 + B_new * u0;
    if has_state_in_obs
        x_next_pred = g_next_pred(end-1:end);
    else
        % todo
    end
    x_next_pred_list(i, :) = x_next_pred';
end

pred_err = x_next_pred_list - x_next_gt_list;
h_err = reshape(abs(pred_err(:, 1)), size(H0));
v_err = reshape(abs(pred_err(:, 2)), size(H0));
fprintf("n_obs: %d, eps: %d\n", size(obs_fun_cell, 1), eps);
fprintf("Average h_err: %d. Min h_err: %d. Max h_err: %d\n", mean(h_err(:)), min(h_err(:)), max(h_err(:)))
fprintf("Average v_err: %d. Min v_err: %d. Max v_err: %d\n\n", mean(v_err(:)), min(v_err(:)), max(v_err(:)))

% plot
figure(1);

subplot(1, 2, 1);
surf(H0, V0, h_err), axis equal, view(2);
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Height Prediction (m)");

subplot(1, 2, 2);
surf(H0, V0, v_err), axis equal, view(2);
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Velocity Prediction (m/s)");

%% Test over trajectory
x0 = [0.3; 0];
sim_steps = 50;

% ground truth
x_traj_gt = zeros(2, sim_steps + 1);
x_traj_gt(:, 1) = x0;
for i = 1:sim_steps
    x_traj_gt(:, i+1) = bouncing_ball_2_dim_dyn(x_traj_gt(:, i), 0);
end

% prediction
x_traj_pred = zeros(2, sim_steps + 1);
x_traj_pred(:, 1) = x0;
for i = 1:sim_steps
    x_curr = x_traj_pred(:, i);
    g_curr = g_list_fun(x_curr, obs_fun_cell);
    g_next_pred = A * g_curr;
    x_next_pred = g_next_pred(end-1:end);
    x_traj_pred(:, i+1) = x_next_pred;
end

% plot
figure(2);

subplot(2, 1, 1);
plot((0:sim_steps) * 10, x_traj_gt(1, :), '-o', 'LineWidth', 4), axis tight, grid on, hold on;
plot((0:sim_steps) * 10, x_traj_pred(1, :), '-o', 'LineWidth', 4);
xlabel("Time (ms)"), ylabel("Height (m)")
title("Height")

subplot(2, 1, 2);
plot((0:sim_steps) * 10, x_traj_gt(2, :), '-o', 'LineWidth', 4), axis tight, grid on, hold on;
plot((0:sim_steps) * 10, x_traj_pred(2, :), '-o', 'LineWidth', 4);
xlabel("Time (ms)"), ylabel("Velocity (m/s)")
title("Velocity")

%% Test LQR
Q = eye(size(A_new, 2));
R = 0.000001;
N = 0;
K_gain = dlqr(A_new, B_new, Q, R, N);

Q_ = eye(2);
A_ = [1, 0.01; 0, 1];
B_ = [0.5 * 0.01^2; 0.01];
K_gain_ = dlqr(A_, B_, Q_, R, N);

%%
figure(4);
subplot(2, 1, 1);

for k = 1:10
    x0 = [rand * 0.5; rand * 0.2 - 0.2];
    g0 = g_list_fun(x0, obs_fun_cell(1:end-1)); % [rbf; x]
    sim_steps = 1000;
    
    x_list = zeros(sim_steps+1, 2);
    x_list_ = zeros(sim_steps+1, 2);
    g_list = zeros(sim_steps+1, size(A_new, 2));
    
    x_list(1, :) = x0';
    x_list_(1, :) = x0';
    g_list(1, :) = g0';
    
    for i = 1:sim_steps
        cur_x = x_list(i, :)';
        cur_x_ = x_list_(i, :)';
        cur_g = g_list(i, :)';
    
        u = -K_gain * cur_g;
        u_ = -K_gain_ * cur_x_;
        x_next = bouncing_ball_2_dim_dyn(cur_x, u);
        x_next_ = A_ * cur_x_ + B_ * u_;
        g_next = g_list_fun(x_next, obs_fun_cell(1:end-1));
    
        x_list(i+1, :) = x_next';
        x_list_(i+1, :) = x_next_';
        g_list(i+1, :) = g_next';
    end
    plot((0:sim_steps) * 10, x_list(:, 1), '-o', 'LineWidth', 2), grid on, hold on;
end

% plot((0:sim_steps) * 10, x_list_(:, 1), '-o', 'LineWidth', 4), axis tight;
xlabel("Time (ms)"), ylabel("Height (m)");
title("LQR with lifted dynamics");
subplot(2, 1, 2);
plot((0:sim_steps) * 10, x_list(:, 2), '-o', 'LineWidth', 4), grid on, hold on;
% plot((0:sim_steps) * 10, x_list_(:, 2), '-o', 'LineWidth', 4), axis tight;
xlabel("Time (ms)"), ylabel("Velocity (m/s)");

%%
function g_value = g_list_fun(x, obs_fun_cell)
num_obs = size(obs_fun_cell, 2);
g_value = zeros(num_obs, 1);
for i = 1:num_obs
    g_value(i) = obs_fun_cell{i}(x);
end
end