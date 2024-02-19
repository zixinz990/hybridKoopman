function pre_run(num_rbf, eps)
syms h v u real
x = [h; v];

% Set parameters
dyn_fun = @bouncing_ball_2_dim_dyn;
uniform_step_size = [0.01; 0.01];
traj_step_size = [0.05; 0.05];
var_density_min_step_size = [0.005; 0.005];
var_density_max_step_size = [0.02; 0.02];

% Generate data
[data_var_density, data_var_density_next] = gen_data(dyn_fun, "var_density", ...
    var_density_min_step_size = var_density_min_step_size, ...
    var_density_max_step_size = var_density_max_step_size);

% Delaunay triangulation
data_DT = delaunayTriangulation(data_var_density);

% Generate RBFs
[rbf_fun_list, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_rbf, eps, data_var_density);

% Generate observables
obs_fun_list = rbf_fun_list;
obs_fun_list{end+1} = matlabFunction(h, 'Vars', {x});
obs_fun_list{end+1} = matlabFunction(v, 'Vars', {x});
num_obs = size(obs_fun_list, 2);

% Save
save("pre_run_result.mat", ...
    "obs_fun_list", "rbf_center_list", "num_rbf", "num_obs", "eps", ...
    "data_var_density", "data_var_density_next", "data_DT");
end
