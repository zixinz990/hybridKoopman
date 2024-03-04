close all; clear; clc;
addpath("./data");
addpath(genpath("./functions"));

% load mat file
file_name = "./data/2024_2_23_7_7_bouncing_ball_2_dim_KDE.mat";
load(file_name);

% Get linear system: g_next = A_lift * g + B_lift * u
% The observables [g; u] = [rbf; x; u]
A_lift = A(1:end-1, 1:end-1);
B_lift = A(1:end-1, end);

% Set LQR parameters of the lifted problem
Q_lift = 0 * eye(size(A_lift, 2));
Q_lift(end-1:end, end-1:end) = 1 * eye(2);
R = 0.000001;
N = 0;
K_lift = dlqr(A_lift, B_lift, Q_lift, R, N);

% Set original dynamics
A = [1, 0.01; 0, 1];
B = [0.5 * 0.01^2; 0.01];

% Set LQR parameters for the original problem
Q = eye(2);
K = dlqr(A, B, Q, R, N);

% Initialize plot
sim_steps = 1000;
figure(1);

subplot(2, 3, 1), hold on, grid on, xlim([0, sim_steps * 10]), ylim([0, 0.5]);
title("LQR with Lifted Dynamics", "FontSize", 20), xlabel("Time (ms)", "FontSize", 16), ylabel("Height (m)", "FontSize", 16);

subplot(2, 3, 2), hold on, grid on, xlim([0, sim_steps * 10]), ylim([-1, 1]);
title("LQR with Lifted Dynamics", "FontSize", 20), xlabel("Time (ms)", "FontSize", 16), ylabel("Velocity (m/s)", "FontSize", 16);

subplot(2, 3, 3), hold on, grid on, xlim([0, sim_steps * 10]), ylim([-40, 40]);
title("LQR with Lifted Dynamics", "FontSize", 20), xlabel("Time (ms)", "FontSize", 16), ylabel("Control (N)", "FontSize", 16);

subplot(2, 3, 4), hold on, grid on, xlim([0, sim_steps * 10]), ylim([0, 0.5]);
title("LQR with Original Airborne Dynamics", "FontSize", 20), xlabel("Time (ms)", "FontSize", 16), ylabel("Height (m)", "FontSize", 16);

subplot(2, 3, 5), hold on, grid on, xlim([0, sim_steps * 10]), ylim([-1, 1]);
title("LQR with Original Airborne Dynamics", "FontSize", 20), xlabel("Time (ms)", "FontSize", 16), ylabel("Velocity (m/s)", "FontSize", 16);

subplot(2, 3, 6), hold on, grid on, xlim([0, sim_steps * 10]), ylim([-40, 40]);
title("LQR with Original Airborne Dynamics", "FontSize", 20), xlabel("Time (ms)", "FontSize", 16), ylabel("Control (N)", "FontSize", 16);

% Generate random initial states
x0_list = zeros(2, 10);
for k = 1:10
    x0_list(:, k) = [rand * 0.5; rand * 0.2 - 0.1];
end

% Simulate the result of LQR with lifted dynamics
num_traj = 10;
for k = 1:num_traj
    % Random initial state
    x0 = x0_list(:, k);
    g0 = g_list_fun(x0, obs_fun_list(1:end-1)); % [rbf; x]

    % Initialize variables
    x_list = zeros(sim_steps+1, 2);
    u_list = zeros(sim_steps, 1);
    g_list = zeros(sim_steps+1, size(A_lift, 2));
    x_list(1, :) = x0';
    g_list(1, :) = g0';

    % Dynamics roll out
    for i = 1:sim_steps
        % Get current state and observables
        cur_x = x_list(i, :)';
        cur_g = g_list(i, :)';

        % Calculate optimal control
        cur_u = -K_lift * cur_g;

        % One-step simulate
        x_next = bouncing_ball_2_dim_dyn(cur_x, cur_u);
        g_next = g_list_fun(x_next, obs_fun_list(1:end-1));

        % Store
        x_list(i+1, :) = x_next';
        u_list(i, :) = cur_u * 1 + 10;
        g_list(i+1, :) = g_next';
    end

    % Plot
    subplot(2, 3, 1);
    plot((0:sim_steps)*10, x_list(:, 1), 'LineWidth', 2);
    subplot(2, 3, 2);
    plot((0:sim_steps)*10, x_list(:, 2), 'LineWidth', 2);
    subplot(2, 3, 3);
    plot((0:sim_steps - 1)*10, u_list(:, 1), 'LineWidth', 2);
end

% Simulate the result of LQR with original airborne dynamics
for k = 1:num_traj
    % Random initial state
    x0 = x0_list(:, k);

    % Initialize variables
    x_list = zeros(sim_steps+1, 2);
    u_list = zeros(sim_steps, 1);
    x_list(1, :) = x0';

    % Dynamics roll out
    for i = 1:sim_steps
        % Get current state and observables
        cur_x = x_list(i, :)';

        % Calculate optimal control
        cur_u = -K * cur_x;

        % One-step simulate
        x_next = A * cur_x + B * cur_u;

        % Store
        x_list(i+1, :) = x_next';
        u_list(i, :) = cur_u;
    end

    % Plot
    subplot(2, 3, 4);
    plot((0:sim_steps)*10, x_list(:, 1), 'LineWidth', 2);
    subplot(2, 3, 5);
    plot((0:sim_steps)*10, x_list(:, 2), 'LineWidth', 2);
    subplot(2, 3, 6);
    plot((0:sim_steps - 1)*10, u_list(:, 1), 'LineWidth', 2);
end

%% Functions
function g_value = g_list_fun(x, obs_fun_list)
num_obs = size(obs_fun_list, 2);
g_value = zeros(num_obs, 1);
for i = 1:num_obs
    g_value(i) = obs_fun_list{i}(x);
end
end
