function X = plot_2_dim_traj(X_ref, x0, U, dyn_fun)
% This function is used to plot a trajectory given the initial state, input
% trajectory and dynamics, and compare the result with a reference
arguments
    X_ref{mustBeNumeric}
    x0(2, 1) {mustBeNumeric}
    U(:, 1) {mustBeNumeric}
    dyn_fun
end

N = size(U, 1) + 1;
n = size(x0, 1);
X = zeros(n, N);
X(:, 1) = x0;
for k = 1:N - 1
    xk = X(:, k);
    uk = U(k);
    X(:, k+1) = dyn_fun(xk, uk);
end

figure;

subplot(3, 1, 1);
plot(0:N-1, X(1, :), '-o', 'LineWidth', 2), hold on, grid on;
plot(0:N-1, X_ref(1, :), '--', 'LineWidth', 2);
title("$x_1$", 'FontSize', 16, 'Interpreter', 'latex');
legend("Simulation", "Reference");

subplot(3, 1, 2);
plot(0:N-1, X(2, :), '-o', 'LineWidth', 2), hold on, grid on;
plot(0:N-1, X_ref(2, :), '--', 'LineWidth', 2);
title("$x_2$", 'FontSize', 16, 'Interpreter', 'latex');
legend("Simulation", "Reference");

subplot(3, 1, 3);
plot(0:N-2, U, 'LineWidth', 2), hold on, grid on;
title("Input Trajectory", 'FontSize', 16, 'Interpreter', 'latex');
end
