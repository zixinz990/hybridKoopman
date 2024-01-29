close all; clear;
addpath("./data");

has_state_in_obs = true;
file_name = "./data/2024_1_29_0_6_bouncing_ball_2_dim_DDE.mat";

syms h v real;
x = [h; v];

% load mat file
load(file_name);
g_list_fun = matlabFunction(g_list, 'Vars', {x}); % the input should be a col vector

%% Test over points
% calculate ground truth and prediction
[H0, V0] = meshgrid(0:0.01:1, -3:0.01:3);
x0_list = [H0(:), V0(:)]; % mesh grid to points list, n_points x 2 matrix

x_next_gt_list = zeros(size(x0_list));
x_next_pred_list = zeros(size(x0_list));

for i = 1:size(x0_list, 1)
    % initial state
    x0 = x0_list(i, :)';
    g0 = g_list_fun(x0);
    
    % ground truth
    x_next_gt = bouncing_ball_dynamics(x0, 0);
    x_next_gt_list(i, :) = x_next_gt;

    % prediction
    g_next_pred = A * g0;
    if has_state_in_obs
        x_next_pred = g_next_pred(1:2);
    else
        % todo
    end
    x_next_pred_list(i, :) = x_next_pred';
end

pred_err = x_next_pred_list - x_next_gt_list;
h_err = reshape(abs(pred_err(:, 1)), size(H0));
v_err = reshape(abs(pred_err(:, 2)), size(H0));
fprintf("n_obs: %d, eps: %d\n", size(g_list, 1), eps);
% fprintf("n_obs: %d, eps_1: %d, eps_2: %d\n", size(g_list, 1), eps_1, eps_2);
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
    x_traj_gt(:, i+1) = bouncing_ball_dynamics(x_traj_gt(:, i), 0);
end

% prediction
x_traj_pred = zeros(2, sim_steps + 1);
x_traj_pred(:, 1) = x0;
for i = 1:sim_steps
    x_curr = x_traj_pred(:, i);
    g_curr = g_list_fun(x_curr);
    g_next_pred = A * g_curr;
    x_next_pred = g_next_pred(1:2);
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