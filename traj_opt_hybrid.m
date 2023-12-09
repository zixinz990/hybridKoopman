close all; clear; clc;

addpath("./functions/");
syms k v u real

%% Dynamics
dt = 0.005;
g = -10;
m = 1; % mass, kg
f = v + (g + u / m) * dt;
f_list = f;
x_lb = -1;
x_ub = 1;
f_range = [-1, 1];

%% Observables
n_obs = 3; % must be odd
g_list = sym(zeros(n_obs, 1));
g_list(1) = sym(1);
for n = 1:(n_obs - 1) / 2
    g_list(2*n) = cos(pi*n*v);
    g_list(2*n+1) = sin(pi*n*v);
end
fun_g = matlabFunction(g_list);

%% Orthonormal basis functions
phi_k = exp(pi*1i*k*v);
phi_list = sym(zeros(n_obs, 1));
phi_list(1) = subs(phi_k, k, 0);
for n = 1:n_obs / 2
    phi_list(2*n) = subs(phi_k, k, n);
    phi_list(2*n+1) = subs(phi_k, k, -n);
end

%% Calculate A
As = cal_As_1_dim_sym(phi_list, f_list, f_range);
C = cal_C_1_dim(phi_list, g_list, x_lb, x_ub);
A = (real(C) * real(As) - imag(C) * imag(As)) * real(inv(C)) - ...
    (real(C) * imag(As) + imag(C) * real(As)) * imag(inv(C));
A = 0.5 * A;
fun_A = matlabFunction(A);

%% Trajectory optimization
N = 5;
v_ref_traj = [-0.9; -0.975; 0.9625; 0.8625; 0.7625];
u_max = 20;
% initial_guess.u = [-5; -2.5; -10; -10];
initial_guess.u = [-10; -10; -10; -10] + 10 * rand() - 5;

traj_opt_prob = setup_problem_2(N, v_ref_traj, u_max, n_obs, fun_A, fun_g);
defaultsolver = solvers(traj_opt_prob);
opts = optimoptions(defaultsolver, Display = "iter", Algorithm = "sqp", OptimalityTolerance = 1e-4);
[sol, fval, eflag, output] = solve(traj_opt_prob, initial_guess, Options = opts);
plot_traj(v_ref_traj(1), sol.u, N);

%% FUNCTIONS
function trajectory_prob = setup_problem_2(N, v_ref_traj, u_max, n_obs, fun_A, fun_g)
trajectory_prob = optimproblem;
u = optimvar("u", N-1, "LowerBound", -u_max, "UpperBound", u_max);
g = optimexpr(n_obs, N);

% Reference trajectory
g_ref_traj = zeros(n_obs, N);
for i = 1:N
    g_ref_traj(:, i) = fun_g(v_ref_traj(i));
end

% Dynamics
g(:, 1) = g_ref_traj(:, 1); % initial condition
for i = 2:N
    g(:, i) = fun_A(u(i-1)) * g(:, i-1);
end

% Objective
cost = optimexpr(1);
for i = 1:N
    cost = cost + norm(g(:, i)-g_ref_traj(:, i));
end
trajectory_prob.Objective = cost;
end

function plot_traj(v0, u_list, N)
dt = 0.0001;
g = -10;

v_list = zeros(50*(N - 1)+1, 1);
v_list(1) = v0;

u_sim_list = zeros(50*(N - 1), 1);
for i = 1:length(u_sim_list)
    u_sim_list(i) = u_list(ceil(i/50));
end

for k = 1:length(v_list) - 1
    v_list(k+1) = v_list(k) + dt * (u_sim_list(k) + g);
    if v_list(k+1) <= -1
        v_list(k+1) = 1;
    end
end

figure(1);

subplot(2, 1, 1);
plot(0.1*(0:length(v_list)-1), v_list, 'LineWidth', 2), hold on;
plot([0; 5; 7; 7; 10; 15; 20], [-0.9; -0.975; -1; 1; 0.9625; 0.8625; 0.7625], 'LineWidth', 2, 'LineStyle', '--');
xlabel("Time (ms)");
ylabel("Velocity (m/s)");
ylim([-1.2, 1.2]);
legend("Trajectory", "Reference Trajectory");

grid on;

subplot(2, 1, 2);
plot(0.1*(0:length(u_sim_list)-1), u_sim_list, 'LineWidth', 2);
xlabel("Time (ms)");
ylabel("Jet Force (N)");
ylim([-11, -2]);
grid on;

grid on;
end