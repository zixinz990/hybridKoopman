close all; clear; clc;
addpath("./data");
addpath(genpath("./functions"));

%% Load Koopman dynamics model
file_name = "./data/2024_3_5_21_56_bouncing_ball_2_dim_KDE.mat";
load(file_name);
rbf_fun_list = obs_fun_cell(1:end-3);
x_dim = 2;
g_dim = num_rbf + x_dim; % observables g = [rbf; x]

A_lift = A(1:end-1, 1:end-1);
B_lift = A(1:end-1, end);
C_lift = zeros(g_dim, 1);

test_num = 1;
position_err = 0;
velocity_err = 0;
while test_num < 10
    %% Trajectory optimization parameters
    horizon = 50;
    dyn_fea_ref = true;
    
    %% Generate reference state trajectory
    % u_min = -5;
    % u_max = 10;
    X_ref = zeros(x_dim, horizon);
    x0 = [0.2; 0];
    X_ref(:, 1) = x0;
    if dyn_fea_ref
        U_ref = rand(horizon-1, 1) * (u_max - u_min) + u_min;
        for h = 1:horizon-1
            xh = X_ref(:, h);
            uh = U_ref(h);
            X_ref(:, h+1) = bouncing_ball_2_dim_dyn(xh, uh);
        end
    else
        U_ref = zeros(horizon - 1, 1);
        X_ref(1, :) = 0.3;
        X_ref(2, :) = 0;
    end
    
    %% Generate reference lifted state trajectory
    G_ref = zeros(g_dim, horizon);
    for h = 1:horizon
        xh_ref = X_ref(:, h);
        G_ref(:, h) = cal_g_val(xh_ref, rbf_fun_list);
    end
    
    %% Setup trajectory optimization problem
    Q = 1.0 * eye(g_dim);
    Q(end-1:end, end-1:end) = diag([10,10]);
    Q_terminal = Q;
    
    R = 0.000001;
    init_guess.U = U_ref;
    % init_guess.U = zeros(horizon-1, 1);
    traj_opt_prob = setup_linear_traj_prob(horizon, G_ref(:, 1), G_ref, ...
                                           u_min, u_max, ...
                                           A_lift, B_lift, C_lift, ...
                                           Q, Q_terminal, R);
    defaultsolver = solvers(traj_opt_prob);
    opts = optimoptions(defaultsolver, Display = "iter", Algorithm = "active-set", OptimalityTolerance = 1e-4);
    [sol, fval, eflag, output] = solve(traj_opt_prob, init_guess, Options = opts);
    X = plot_2_dim_traj(X_ref, x0, sol.U, @bouncing_ball_2_dim_dyn);
    subplot(3, 1, 3);
    plot(0:horizon-2, U_ref, '--', 'LineWidth', 2), hold on, grid on;
    legend("Solution", "Truth");
    
    %% Calculate Error
    position_err = position_err + norm(X(1, :) - X_ref(1, :));
    velocity_err = velocity_err + norm(X(2, :) - X_ref(2, :));
    
    test_num = test_num + 1;
end

%% Functions
function g_val = cal_g_val(x, rbf_fun_list)
obs_dim = size(rbf_fun_list, 2);
g_val = zeros(obs_dim + 2, 1);
for i = 1:obs_dim
    gi_fun = rbf_fun_list{i};
    g_val(i) = gi_fun(x);
end
g_val(end-1:end) = x;
end