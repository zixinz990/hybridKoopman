close all; clear; clc;
syms h v u real
x = [h; v];
dyn_fea_ref = false;

% Set linear dynamics
dt = 0.01;
A = [1, dt; 0, 1];
B = [0.5 * dt^2; dt];
C = -[0.5*10*dt^2; 10*dt];
dyn_fun = matlabFunction(A*x + B*u + C, "Vars", {x, u});

% Set parameters
N = 100;
x0 = [10; 0];
u_min = -50;
u_max = 50;
Q = diag([10, 1]);
Q_terminal = 100 * diag([10, 1]);
R = 0.000001;

% Reference state trajectory
X_ref = zeros(size(x0, 1), N);
if dyn_fea_ref
    % Generate a dynamical feasible random reference state trajectory
    X_ref(:, 1) = x0;
    for k = 1:N - 1
        xk = X_ref(:, k);
        uk = rand * (u_max - u_min) + u_min;
        X_ref(:, k+1) = A * xk + B * uk + C;
    end
else
    % Generate a dynamical infeasible reference state trajectory    
    for k = 1:N
        X_ref(:, k) = [8; 1];
    end
end

% Initial guess
init_guess.U = 0 * ones(N-1, 1);

% Solve
traj_opt_prob = setup_linear_traj_prob(N, x0, X_ref, u_min, u_max, A, B, C, Q, Q_terminal, R);
defaultsolver = solvers(traj_opt_prob);
opts = optimoptions(defaultsolver, Display = "iter", Algorithm = "active-set", OptimalityTolerance = 1e-4);
[sol, fval, eflag, output] = solve(traj_opt_prob, init_guess, Options = opts);
plot_2_dim_traj(X_ref, x0, sol.U, dyn_fun);
