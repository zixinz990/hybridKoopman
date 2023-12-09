close all; clear; clc;

syms rho mu x1 x2 u

% Dynamics
f = [rho * x1; mu * (x2 - x1^2)];
g = [1; 0];

% Eigenvalues and eigenfunctions
lambda1 = rho;
lambda2 = mu;
lambda3 = 2 * rho;

phi1 = x1;
phi2 = x2 - (mu / (mu - 2 * rho)) * x1^2;
phi3 = x1^2;

z = [phi1; phi2; phi3];

% Koopman mode
vx1 = [1; 0];
vx2 = [0; 1];
vx3 = [0; mu / (mu - 2 * rho)];

Cx = [vx1, vx2, vx3];

% Final Koopman observer form
Lambda = diag([rho, mu, 2 * rho]);
g_ = jacobian(z, [x1, x2]) * g;

dzdt = Lambda * z + g_ * u;

% Test
rho_ = 1;
mu_ = 1;

f = subs(f, [rho, mu], [rho_, mu_]);
g = subs(g, [rho, mu], [rho_, mu_]);
dxdt = f + g * u;
dzdt = subs(dzdt, [rho, mu], [rho_, mu_]);

dt = 0.001;
x0 = [0.3; -1];
sim_length = 500;

% Ground truth
x_list = zeros(2, sim_length);
x_list(:, 1) = x0;
for k = 1:sim_length-1
    u_ = sin(2*pi*dt*(k - 1));
    x1_ = x_list(1, k);
    x2_ = x_list(2, k);
    dxdt_ = subs(dxdt, [x1, x2, u], [x1_, x2_, u_]);
    x_list(:, k+1) = x_list(:, k) + double(dxdt_*dt);
end

% Estimation
z_list = zeros(3, sim_length);
z_list(:, 1) = double(subs(z, [x1, x2, mu, rho], [x0(1), x0(2), mu_, rho_]));
x_est_list = zeros(2, sim_length);
x_est_list(:, 1) = x0;
for k = 1:sim_length-1
    u_ = sin(2*pi*dt*(k - 1));
    x1_ = x_est_list(1, k);
    x2_ = x_est_list(2, k);
    dzdt_ = subs(dzdt, [x1, x2, u], [x1_, x2_, u_]);
    z_list(:, k+1) = z_list(:, k) + double(dzdt_*dt);
    Cx_ = subs(Cx, [mu, rho], [mu_, rho_]);
    x_est_list(:, k+1) = Cx_ * z_list(:, k+1);
end

%% Plot
subplot(2, 1, 1);
plot(x_list(1, :), 'LineWidth', 2), hold on;
plot(x_est_list(1, :), 'LineStyle', ':', 'LineWidth', 4);
legend("true", "estimate");
grid on;

subplot(2, 1, 2);
plot(x_list(2, :), 'LineWidth', 2), hold on;
plot(x_est_list(2, :), 'LineStyle', ':', 'LineWidth', 4);
legend("true", "estimate");
grid on;
