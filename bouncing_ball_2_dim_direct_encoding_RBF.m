close all; clear; clc;

syms x0 v0 xk vk t1 real

g = -9.81;
dt = 0.001;
v_max_fun = matlabFunction(sqrt(v0^2 - 2*g*x0)); % @(v0, x0)
x_max_fun = matlabFunction((2*g*x0 - v0^2) / (2*g)); % @(v0, x0)
state = [vk; xk];

% Airborne dynamics
f1 = [vk + g*dt; ...
      xk + vk*dt + 0.5*g*dt^2];
f1_fun = matlabFunction(f1); % @(vk,xk)

% Jump dynamics
bc = xk + vk*t1 + 0.5*g*t1^2 == 0;
jump_cond_1 = matlabFunction(xk + vk*dt + 0.5*g*dt^2 <= 0);
jump_cond_2 = matlabFunction(vk < 0);
t1 = solve(bc, t1);
t1 = t1(2);
t2 = simplify(dt - t1);
v_contact = simplify(vk + g*t1);
f2 = [simplify(-v_contact + g*t2); ...
      simplify(-v_contact*t2 + 0.5*g*t2^2)];
f2_fun = matlabFunction(f2); % @(vk,xk)

%% Simulate
x0_ub = 0.5;
v0 = 0;
x0 = 0 : 0.05 : x0_ub;
v_max = v_max_fun(v0, x0(end));

state_init_list = [];
for i = 1:length(v0)
    for j = 1:length(x0)
        if v0(i) == 0 && x0(j) == 0
            state_init_list = state_init_list;
        else
            state_init_list = [state_init_list, [v0(i); x0(j)]];
        end
    end
end

state_sim_list = [];
sim_steps = 800;
for i = 1:size(state_init_list, 2)
    state_sim_list_tmp = zeros(2, sim_steps+1);
    state_sim_list_tmp(:, 1) = state_init_list(:, i);
    for k = 1:sim_steps
        need_jump = jump_cond_1(state_sim_list_tmp(1, k), state_sim_list_tmp(2, k)) && jump_cond_2(state_sim_list_tmp(1, k));
        if need_jump
            state_sim_list_tmp(:, k+1) = f2_fun(state_sim_list_tmp(1, k), state_sim_list_tmp(2, k));
        else
            state_sim_list_tmp(:, k+1) = f1_fun(state_sim_list_tmp(1, k), state_sim_list_tmp(2, k));
        end
    end
    state_sim_list = [state_sim_list, state_sim_list_tmp];
%     plot(state_sim_list_tmp(1, :), state_sim_list_tmp(2, :)), hold on;
end
% axis equal;

%% Choose RBFs
n_obs = 750;
eps = 15;
radius = -log(0.5) / eps;
[idx, C] = kmeans(state_sim_list', n_obs, 'MaxIter', 200);

% figure(2);
% for k = 1:n_obs
%     viscircles([C(k, 1), C(k, 2)], radius);
% end
% axis equal;