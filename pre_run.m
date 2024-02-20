function pre_run(num_rbf, eps)
% eps: 15, 20, 25
syms h v u real
x = [h; v];

% Set parameters
dyn_fun = @bouncing_ball_2_dim_dyn;
uniform_step_size = [0.015; 0.015];
traj_step_size = [0.05; 0.05];
var_density_min_step_size = [0.005; 0.005];
var_density_max_step_size = [0.02; 0.02];

% Generate data
% data contains state and 0 control
[data, data_next] = gen_data(dyn_fun, "uniform", ...
    uniform_step_size=uniform_step_size);
% [data_var_density, data_var_density_next] = gen_data(dyn_fun, "var_density", ...
%     var_density_min_step_size = var_density_min_step_size, ...
%     var_density_max_step_size = var_density_max_step_size);

% Delaunay triangulation
data_DT = delaunayTriangulation(data(:, 1:2));

% Generate RBFs
[rbf_fun_list, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_rbf, eps, data(:, 1:2));

% Plot
figure(1);
subplot(1, 3, 1);
plot(data(:, 1), data(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling", "FontSize", 24)
axis equal, grid on;
subplot(1, 3, 2);
plot(data_next(:, 1), data_next(:, 2), '.');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Uniform Sampling (next)", "FontSize", 24)
axis equal, grid on;
subplot(1, 3, 3);
plot(rbf_center_list(:, 1), rbf_center_list(:, 2), 'o');
xlabel("Height (m)", 'FontSize', 16);
ylabel("Velocity (m/s)", "FontSize", 16);
title("Center of Observables", "FontSize", 24)
axis equal; grid on;

% Generate observables
obs_fun_list = rbf_fun_list;
obs_fun_list{end+1} = matlabFunction(h, 'Vars', {x});
obs_fun_list{end+1} = matlabFunction(v, 'Vars', {x});
obs_fun_list{end+1} = matlabFunction(u, 'Vars', {[x; u]});
num_obs = size(obs_fun_list, 2);

% Save
save("pre_run_result.mat", ...
    "obs_fun_list", "rbf_center_list", "num_rbf", "num_obs", "eps", ...
    "data", "data_next", "data_DT");
end
