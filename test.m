close all; clear;

global k x
syms k x real;

% dynamics
a = 0.31; b = 0.94; c = -3;
x_star = 0.32;

F1 = c*x - c*x_star;
F2 = b*x + a - b*x_star;

% ground truth
states = zeros(100, 1); g2 = zeros(100, 1);
states(1) = 0; g2(1) = double(subs(cos(2*pi*x), x, 0));

for t = 2:100
    if states(t-1) <= x_star
        states(t) = subs(F1, x, states(t-1));
    else
        states(t) = subs(F2, x, states(t-1));
    end
    g2(t) = double(subs(cos(2*pi*x), x, states(t)));
end

subplot(3, 1, 1)
plot(1:100, states), hold on

subplot(3, 1, 2)
plot(1:100, tan(2*pi*states));

subplot(3, 1, 3)
plot(1:100, atan(tan(2*pi*states)))
% plot(1:100, g2, 'LineWidth', 2), hold on;
% g2_predict = predict(F1, F2, 17);
% plot(1:100, g2_predict, 'LineWidth', 2);
% xlabel("step"), ylabel("g_2(x)"), legend("Ground Truth", "Prediction")
% title("m = 17")

% subplot(3, 1, 2)
% plot(1:100, g2, 'LineWidth', 2), hold on;
% g2_predict = predict(F1, F2, 33);
% plot(1:100, g2_predict, 'LineWidth', 2);
% xlabel("step"), ylabel("g_2(x)"), legend("Ground Truth", "Prediction")
% title("m = 33")

subplot(3, 1, 3)
plot(1:100, g2, 'LineWidth', 2), hold on;
g2_predict = predict(F1, F2, 257);
plot(1:100, g2_predict, 'LineWidth', 2);
xlabel("step"), ylabel("g_2(x)"), legend("Ground Truth", "Prediction")
title("m = 257")

function g2_predict = predict(F1, F2, m)
global k x
syms k x real
% m should be odd
% observables
X(1) = sym(1);
for n = 1:(m-1)/2
    X(2*n) = cos(2*pi*n*x); X(2*n+1) = sin(2*pi*n*x);
end

% orthonormal basis functions
phi_k = exp(2*pi*1i*k*x);
Phi(1) = subs(phi_k, k, 0);
for n = 1:m/2
    Phi(2*n) = subs(phi_k, k, n); Phi(2*n+1) = subs(phi_k, k, -n);
end

% composed functions phi_k[F(x)]
Phi_F1 = subs(Phi, x, F1);
Phi_F2 = subs(Phi, x, F2);

A_simple = cal_A_simple(m);
C = cal_C(m);
C_inv = cal_C_inv(m);
A = C * A_simple * C_inv;

% prediction
X_predict = zeros(m, 100); g2_predict = zeros(100, 1);

X_predict(:, 1) = double(subs(X, x, 0));
g2_predict(1) = X_predict(2, 1);

for t = 2:100
    X_predict(:, t) = A * X_predict(:, t-1);
    g2_predict(t) = X_predict(2, t);
end
end
