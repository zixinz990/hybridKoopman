function run(num_rbf, eps, u_min, u_max)
%% Sample data
dyn_fun = @bouncing_ball_2_dim_dyn;
sample_method = "uniform";
uniform_step_size = [0.015; 0.015];
% In the data matrix, each row is [x1, x2, u]
% Extend the bound to make more observables cover the boundaries
h_lb_extend = 0.2;
h_ub_extend = 0.2;
v_lb_extend = 0.2;
v_ub_extend = 0.2;
[data, data_extended, data_next, x_range] = gen_data(dyn_fun, sample_method, ...
                                            uniform_step_size=uniform_step_size, ...
                                            u_min=u_min, u_max=u_max, ...
                                            h_lb_extend=h_lb_extend, h_ub_extend=h_ub_extend, ...
                                            v_lb_extend=v_lb_extend, v_ub_extend=v_ub_extend);
fprintf("Number of data: %d\n", size(data, 1));
x_range_extend = x_range + [-h_lb_extend, h_ub_extend; -v_lb_extend, v_ub_extend];

%% Delaunay triangulation
data_DT = delaunayTriangulation(data(:, 1:2));

%% Generate RBFs
% Use extended dataset to cover the h=0 boundry better
[rbf_fun_cell, rbf_centers] = gen_gaussian_rbfs_2_dim(num_rbf, eps, data_extended(:, 1:2));

%% Generate observable function cell
obs_fun_cell = rbf_fun_cell;
syms h v u real
x = [h; v];
obs_fun_cell{end+1} = matlabFunction(h, 'Vars', {x});
obs_fun_cell{end+1} = matlabFunction(v, 'Vars', {x});
obs_fun_cell{end+1} = matlabFunction(u, 'Vars', {[x; u]});
obs_dim = size(obs_fun_cell, 2);

%% Estimate R matrix
R = zeros(obs_dim, obs_dim);
for i = 1:obs_dim
    parfor j = i:obs_dim
        fprintf('Estimating R matrix: i = %d, j = %d\n', i, j);
        result = est_inner_product(obs_fun_cell{i}, obs_fun_cell{j}, data, data_DT);
        R(i, j) = result;
    end
end
R = R + R' - tril(R);

%% Estimate Q matrix
Q = zeros(obs_dim, obs_dim);
for i = 1:obs_dim
    parfor j = 1:obs_dim
        if j == 5
            disp("")
        end
        fprintf('Estimating Q matrix: i = %d, j = %d\n', i, j);
        result = est_inner_product(obs_fun_cell{i}, obs_fun_cell{j}, data_next, data_DT);
        Q(i, j) = result;
    end
end

%% Get A matrix
A = Q * inv(R);

%% Save
time_arr = string(clock);
under_line_arr = repmat("_", size(time_arr));
time_arr = [time_arr; under_line_arr];
time_arr = time_arr(:)';
time_arr = time_arr(1:end-2);
file_name = "./data/" + strjoin(time_arr, '') + "bouncing_ball_2_dim_KDE.mat";
save(file_name, "R", "Q", "A", ...
                "num_rbf", "eps", "u_min", "u_max", "sample_method", "uniform_step_size", ...
                "rbf_centers", "obs_dim", "obs_fun_cell", "x_range_extend");

%% Plot
figure(1);

subplot(1, 4, 1);
plot(data(:, 1), data(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
ylim([-4, 4]);
title("Uniform Sampling", "FontSize", 24);
axis equal, grid on;

subplot(1, 4, 2);
plot(data_next(:, 1), data_next(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling (next)", "FontSize", 24);
ylim([-4, 4]);
axis equal, grid on;

subplot(1, 4, 3);
triplot(data_DT);
ylim([-4, 4]);
axis equal, grid on;

subplot(1, 4, 4);
rbf_08_radius = sqrt(-log(0.2) / eps^2); % radius of 20%
hold on;
for i = 1:num_rbf
    ci = rbf_centers(i, :);
    draw_circle(ci(1), ci(2), rbf_08_radius);
end
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables with 20% radius", "FontSize", 24);
ylim([-4, 4]);
axis equal; grid on;

% left bound of sample area
plot([x_range(1, 1), x_range(1, 1)], [x_range(2, 1), x_range(2, 2)], 'LineWidth', 2, 'Color', 'black');
% right bound of sample area
plot([x_range(1, 2), x_range(1, 2)], [x_range(2, 1), x_range(2, 2)], 'LineWidth', 2, 'Color', 'black');
% upper bound of sample area
plot([x_range(1, 1), x_range(1, 2)], [x_range(2, 2), x_range(2, 2)], 'LineWidth', 2, 'Color', 'black');
% lower bound of sample area
plot([x_range(1, 1), x_range(1, 2)], [x_range(2, 1), x_range(2, 1)], 'LineWidth', 2, 'Color', 'black');

end
