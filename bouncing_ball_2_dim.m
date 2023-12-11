close all; clear; clc;

addpath("./functions/");
syms x xdot real

%% Parameters
dt = 0.001;
g = -9.807;

v_max = 0.5;
x_max = -v_max^2 / (2 * g);

sim_steps = 412;
n_obs = 52;
eps = 30; % parameter of RBF
radius = -log(0.5) / eps;

plot_rbf_pos = false;

%% Dynamics
f1 = [x + xdot * dt + 0.5 * g * dt^2; ...
      xdot + g * dt];

t1 = (-v_max - xdot) / g;
t2 = dt - t1;
f2 = [simplify(v_max * t2 + 0.5 * g * t2^2);
      xdot + g * dt + 1];

f1_fun = matlabFunction(f1);
f2_fun = matlabFunction(f2);
f_list = [f1; f2];

%% Simulate
state_init = [0; 0.5];
state_sim_list = zeros(sim_steps+1, 2);
state_sim_list(1, :) = state_init';
for i = 1:sim_steps
    x_cur = state_sim_list(i, 1);
    v_cur = state_sim_list(i, 2);
    if x_cur == 0 && v_cur < 0
        state_sim_list(i + 1, 1) = 0;
        state_sim_list(i + 1, 2) = -v_cur;
    else
        state_sim_list(i + 1, :) = f1_fun(x_cur, v_cur);
        if state_sim_list(i + 1, 1) < 0
            state_sim_list(i + 1, 1) = 0;
            state_sim_list(i + 1, 2) = -sqrt(-2*g*x_cur+v_cur^2);
        end
    end
end

% %% Determine Centers of RBF Observables using k-Means
% % [idx, C] = kmeans(state_sim_list, n_obs);
% C = state_sim_list(1:2:104, :);
% C = sortrows(C, 2);
% if plot_rbf_pos
%     figure(1);
%     plot(state_sim_list(:, 2), state_sim_list(:, 1)), hold on;
%     for k = 1:n_obs
%         viscircles([C(k, 2), C(k, 1)], radius);
%     end
%     xlabel("Velocity"), ylabel("Height");
%     xlim([-v_max, v_max]);
%     axis equal;
% end
% 
% %% RBF Observables
% g_list = sym(zeros(n_obs, 1));
% for k = 1:n_obs    
%     g_list(k) = exp(-eps * norm([x, xdot] - C(k, :)));
% end
% g_list_fun = matlabFunction(g_list);
% 
% %% Calculate Q and R
% % g_list_xdot = subs(g_list, x, (xdot^2 - v_max^2) / (2 * g));
% % 
% % % Compose g and f
% % % g_f_list(i, j) = gi(fj(x))
% % g_f_list_xdot = sym(zeros(n_obs, 1));
% % for i = 1:n_obs
% %     gi_f = subs(g_list(i), [x; xdot], f);
% %     g_f_list_xdot(i) = subs(gi_f, x, (xdot^2 - v_max^2) / (2 * g)); 
% % end
% % 
% % Q = cal_Q_bouncing_ball(g_list_xdot, g_f_list_xdot, -v_max, v_max);
% % R = cal_R_bouncing_ball(g_list_xdot, -v_max, v_max);
% % A = Q * inv(R);
% load("A.mat")
% 
% %% Predict
% g_init = g_list_fun(state_init(1), state_init(2));
% 
% g_predict_list = zeros(n_obs, sim_steps+1);
% g_predict_list(:, 1) = g_list_fun(state_init(1), state_init(2));
% 
% state_predict_list = zeros(sim_steps+1, 2);
% state_predict_list(1, :) = state_init';
% 
% for i = 1:sim_steps
%     g_predict_list(:, i+1) = A * g_predict_list(:, i);
%     [max_g, idx] = max(g_predict_list(:, i+1));
%     g_predict_list(:, i+1) = g_predict_list(:, i+1) / max_g;
%     state_predict_list(i+1, :) = C(idx, :);
% end
% 
% figure(2);
% % plot(state_sim_list(:, 1)), hold on;
% subplot(2, 1, 1);
% plot(state_predict_list(:, 1));
% subplot(2, 1, 2);
% plot(state_predict_list(:, 2));