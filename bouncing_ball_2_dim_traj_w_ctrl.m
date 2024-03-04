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

% Set original dynamics
A = [1, 0.01; 0, 1];
B = [0.5 * 0.01^2; 0.01];

% Generate some random initial states
num_traj = 2;
x0_list = zeros(2, num_traj);
for k = 1:num_traj
    x0_list(:, k) = [rand * 0.5; rand * 0.2 - 0.1];
end

% Simulate with some random control
figure(1);
subplot(2, 2, 1), xlabel("Time (ms)", 'FontSize', 16), ylabel("Height (m)", 'FontSize', 16), hold on, grid on;
subplot(2, 2, 2), xlabel("Time (ms)", 'FontSize', 16), ylabel("Height (m)", 'FontSize', 16), hold on, grid on;
subplot(2, 2, 3), xlabel("Time (ms)", 'FontSize', 16), ylabel("Velocity (m/s)", 'FontSize', 16), hold on, grid on;
subplot(2, 2, 4), xlabel("Time (ms)", 'FontSize', 16), ylabel("Velocity (m/s)", 'FontSize', 16), hold on, grid on;
sim_steps = 200;
for k = 1:num_traj
    % Random initial state
    x0 = x0_list(:, k);
    g0 = g_list_fun(x0, obs_fun_list(1:end-1)); % [rbf; x]

    % Initialize variables
    x_list_lift = zeros(sim_steps+1, 2); % store state from lifted dynamics
    x_list = zeros(sim_steps+1, 2); % store state from original dynamics
    u_list = zeros(sim_steps, 1); % store some random control input
    for i = 1:sim_steps
        u_list(i) = rand * 20 - 10;
    end
    g_list = zeros(sim_steps+1, size(A_lift, 2)); % store observables
    x_list_lift(1, :) = x0';
    x_list(1, :) = x0';
    g_list(1, :) = g0';

    % Dynamics roll out
    for i = 1:sim_steps
        % Get current state, observables and control
        cur_x_lift = x_list_lift(i, :)';
        cur_x = x_list(i, :)';
        cur_g = g_list(i, :)';
        cur_u = u_list(i);

        % One-step simulate
        g_next = A_lift * cur_g + B_lift * cur_u;
        x_next_lift = g_next(end-1:end);
        x_next = bouncing_ball_2_dim_dyn(cur_x, cur_u);

        % Store
        x_list_lift(i+1, :) = x_next_lift';
        x_list(i+1, :) = x_next;
        g_list(i+1, :) = g_next';
    end

    % Plot
    subplot(2, 2, k);
    plot((0:sim_steps)*10, x_list_lift(:, 1), 'LineWidth', 2);
    plot((0:sim_steps)*10, x_list(:, 1), 'LineWidth', 2);
    legend("Lifted Dynamics", "Ground Truth", 'FontSize', 12);
    subplot(2, 2, k+2);
    plot((0:sim_steps)*10, x_list_lift(:, 2), 'LineWidth', 2);
    plot((0:sim_steps)*10, x_list(:, 2), 'LineWidth', 2);
    legend("Lifted Dynamics", "Ground Truth", 'FontSize', 12);
end

%% Functions
function g_value = g_list_fun(x, obs_fun_list)
num_obs = size(obs_fun_list, 2);
g_value = zeros(num_obs, 1);
for i = 1:num_obs
    g_value(i) = obs_fun_list{i}(x);
end
end
