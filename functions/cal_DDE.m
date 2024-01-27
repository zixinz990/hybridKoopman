function cal_DDE(n_obs_1, n_obs_2, eps_1, eps_2)
syms h v real
x = [h; v];

n_obs = n_obs_1 + n_obs_2;
g = -9.81;
h_min = 0; % must be zero
h0_max = 1;
v0_max = 2;
v_max = sqrt(-2*g*h0_max+v0_max^2);
v_min = -v_max;
h_max = h0_max - 0.5 * v0_max^2 / g;
h_jump = 0.1; % approximated mode 2 bound

% sample
[H1, V1] = meshgrid(h_jump:0.1:h_max, v_min:0.1:v_max+0.1);
[H2, V2] = meshgrid(h_min:0.01:h_jump, v_min:0.01:v_max); % approximated mode 2 reagion

state_points_1 = [H1(:), V1(:)]; % mode 1
state_points_2 = [H2(:), V2(:)]; % mode 2, contact
state_points = unique([state_points_1; state_points_2], 'rows');

state_next_points = zeros(size(state_points));
for i = 1:size(state_points, 1)
    state_next = bouncing_ball_dynamics(state_points(i, :)', 0);
    state_next_points(i, :) = state_next';
end
state_DT = delaunayTriangulation(state_points);

% figure(1);
% triplot(state_DT), axis equal, grid on;

% RBF observables
[~, rbf_center_list_1] = kmeans(unique(state_points_1, 'rows'), n_obs_1); % mode 1
[~, rbf_center_list_2] = kmeans(unique(state_points_2, 'rows'), n_obs_2); % mode 2
rbf_center_list = [rbf_center_list_1; rbf_center_list_2];

g_list = sym(zeros(n_obs, 1));
for k = 1:n_obs_1
    rbf_center = rbf_center_list(k, :);
    g_list(k) = exp(-eps_1^2*(x - rbf_center')'*(x - rbf_center'));
end
for k = n_obs_1 + 1:n_obs
    rbf_center = rbf_center_list(k, :);
    g_list(k) = exp(-eps_2^2*(x - rbf_center')'*(x - rbf_center'));
end

% figure(2);
% plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o'), axis equal, grid on;

g_list = [x; g_list]; % add state into observables
n_obs = n_obs + 2;

% estimate R
R = zeros(n_obs, n_obs);
for i = 1:n_obs
    parfor j = i:n_obs
        fprintf('Estimate R matrix: i = %d, j = %d\n', i, j);
        gi_fun = matlabFunction(g_list(i), 'Vars', {x});
        gj_fun = matlabFunction(g_list(j), 'Vars', {x});
        result = estimate_inner_product(gi_fun, gj_fun, state_points, state_DT);
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
        result = estimate_inner_product(gi_fun, gj_fun, state_next_points, state_DT);
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
file_name = "./data/" + strjoin(time_arr, '') + "bouncing_ball_2_dim_DDE.mat";
save(file_name, "R", "Q", "A", "rbf_center_list", "n_obs_1", "n_obs_2", "eps_1", "eps_2", "g_list");

end