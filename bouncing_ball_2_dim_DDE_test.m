close all; clear; clc;

has_state_in_obs = true;
file_name = "2024_0123_2230_bouncing_ball_2_dim_DDE.mat";

syms h v real;
x = [h; v];

% load mat file
load(file_name);
g_list_fun = matlabFunction(g_list, 'Vars', {x}); % the input should be a col vector

% calculate ground truth and prediction
[H0, V0] = meshgrid(0:0.1:10, -5:0.1:5);
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

% plot
subplot(2, 1, 1);
surf(H0, V0, h_err), axis equal, view(2);
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Height Prediction (m)");

subplot(2, 1, 2);
surf(H0, V0, v_err), axis equal, view(2);
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Velocity Prediction (m/s)");
