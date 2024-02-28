close all; clear; clc;
addpath("./data");
addpath(genpath("./functions"));

% load mat file
file_name = "./data/2024_2_20_9_2_bouncing_ball_2_dim_KDE.mat";
load(file_name);

% Get linear system: g_next = A_lift * g + B_lift * u
% The observables [g; u] = [rbf; x; u]
A_lift = A(1:end-1, 1:end-1);
B_lift = A(1:end-1, end);

% Calculate eigenvalues of A_list
A_lift_eigs = eig(A_lift);

% Plot
p = plot(real(A_lift_eigs), imag(A_lift_eigs), 'o');
hold on;
p.MarkerFaceColor = [0, 0, 0];
p.MarkerEdgeColor = [0, 0, 0];
p.MarkerSize = 2;
fplot(@(t) sin(t), @(t) cos(t), 'LineWidth', 2);
grid on, axis equal;
title("Eigenvalues of the Lifted Discrete-time Dynamics", 'FontSize', 20);

% Controllability test of lifted dynamics
num_lift_states = size(B_lift, 1);
C_lift = zeros(size(B_lift, 1), size(B_lift, 2) * num_lift_states);
for n = 1:num_lift_states
    C_lift(:, n) = A_lift^(n-1) * B_lift;
end
fprintf("Number of lifted states: %d\n", num_lift_states);
fprintf("Rank of C_lift: %d\n\n", rank(C_lift));

% Set original dynamics
A = [1, 0.01; 0, 1];
B = [0.5 * 0.01^2; 0.01];

% Controllability test of original dynamics
num_states = size(B, 1);
C = zeros(size(B, 1), size(B, 2) * num_states);
for n = 1:num_states
    C(:, n) = A^(n-1) * B;
end
fprintf("Number of states: %d\n", num_states);
fprintf("Rank of C: %d\n", rank(C));
