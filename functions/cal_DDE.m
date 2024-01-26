function cal_DDE(n_obs, eps)
addpath("./functions")

syms h v u real
x = [h; v];

h_min = 0; % must be zero
h_max = 5;
v_min = -2.5;
v_max = 2.5;
[H, V] = meshgrid(h_min:0.1:h_max, v_min:0.1:v_max);
state_points = [H(:), V(:)];

state_next_points = zeros(size(state_points));
for i = 1:size(state_points, 1)
    state_next = bouncing_ball_dynamics(state_points(i, :)', 0);
    state_next_points(i, :) = state_next';
end

state_DT = delaunayTriangulation(state_points);
state_IC = incenter(state_DT);
dv = 0.5 * 0.1 * 0.1;

% RBF observables
[~, rbf_center_list] = kmeans(state_points, n_obs);
g_list = sym(zeros(n_obs, 1));
for k = 1:n_obs
    rbf_center = rbf_center_list(k, :);
    g_list(k) = exp(-eps^2*(x - rbf_center')'*(x - rbf_center'));
end
g_list = [x; g_list]; % add state into observables
n_obs = n_obs + 2;

% estimate R
R = zeros(n_obs, n_obs);
f_fun = matlabFunction(x+u, 'Vars', {x, u});
for i = 1:n_obs
    parfor j = i:n_obs
        fprintf('Estimate R matrix: i = %d, j = %d\n', i, j);
        gi_fun = matlabFunction(g_list(i), 'Vars', {x});
        gj_fun = matlabFunction(g_list(j), 'Vars', {x});
        result = estimate_inner_product(gi_fun, gj_fun, f_fun, ...
                                        state_DT, dv);
        R(i, j) = result;
    end
end
R = R + R' - tril(R);

% estimate Q
Q = zeros(n_obs, n_obs);
for i = 1:n_obs
    parfor j = 1:n_obs
        fprintf('Estimate Q matrix: i = %d, j = %d\n', i, j);
        gi_fun = matlabFunction(g_list(i), 'Vars', {x});
        gj_fun = matlabFunction(g_list(j), 'Vars', {x});
        result = estimate_inner_product(gi_fun, gj_fun, @bouncing_ball_dynamics, ...
                                        state_DT, dv);
        Q(i, j) = result;
    end
end

% calculate lifted linear dynamics
A = Q * inv(R);

% save
time_arr = string(clock);
under_line_arr = repmat("_", size(time_arr));
time_arr = [time_arr; under_line_arr];
time_arr = time_arr(:)';
time_arr = time_arr(1:end-2);
file_name = strjoin(time_arr, '') + "bouncing_ball_2_dim_DDE.mat";
save(file_name, "R", "Q", "A", "rbf_center_list", "eps", "g_list");

end