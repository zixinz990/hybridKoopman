close all; clear;
clc;

syms h v real;
x = [h; v];

load("2024_0122_0810_bouncing_ball_2_dim_DDE.mat");
g_list_fun = matlabFunction(g_list, 'Vars', {x});

[H0, V0] = meshgrid(0:0.1:5, -2.5:0.1:2.5);
x0_list = [H0(:), V0(:)]; % mesh grid to points list
x_next_gt_list = zeros(size(x0_list));
x_next_pred_list = zeros(size(x0_list));
for i = 1:size(x0_list, 1)
    x0 = x0_list(i, :)';
    g0 = g_list_fun(x0);

    x_next_gt = bouncing_ball_dynamics(x0, 0);
    g_next_gt = g_list_fun(x_next_gt);

    g_next_pred = A * g0;

    [max_g_pred, max_g_idx_pred] = max(g_next_pred);
    x_next_pred = rbf_center_list(max_g_idx_pred, :);

    x_next_gt_list(i, :) = x_next_gt';
    x_next_pred_list(i, :) = x_next_pred';    
end

pred_err = x_next_pred_list - x_next_gt_list;
h_err = abs(pred_err(:, 1));
v_err = abs(pred_err(:, 2));

subplot(2, 1, 1);
surf(H0, V0, reshape(h_err, size(H0))), axis equal;
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Height Prediction (m)");
view(2);

subplot(2, 1, 2);
surf(H0, V0, reshape(v_err, size(H0))), axis equal;
colorbar;
xlabel("Height"), ylabel("Velocity");
title("Error of Velocity Prediction (m/s)");
view(2);