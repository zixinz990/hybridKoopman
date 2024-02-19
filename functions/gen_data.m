function [data, data_next] = gen_data(dyn_fun, sample_method, options)
% Generate data for KDE
% Input:
%   dyn_fun:       function handle of dynamics
%   sample_method: method of sampling
% Output:
%   data:      points of current state
%   data_next: move all points in data one step forward
arguments
    dyn_fun
    sample_method{mustBeText}
    options.uniform_step_size(2, 1) {mustBeNumeric, mustBePositive} = [0.01; 0.01]
    options.traj_step_size(2, 1) {mustBeNumeric, mustBePositive} = [0.05; 0.05]
    options.var_density_min_step_size(2, 1) {mustBeNumeric, mustBePositive} = [0.005; 0.005]
    options.var_density_max_step_size(2, 1) {mustBeNumeric, mustBePositive} = [0.02; 0.02]
end

g = 10;

if sample_method == "uniform"
    % Setup dynamical range
    h_min = 0; % must be zero
    h0_max = 0.5;
    v0_max = 0.5;
    v_max = sqrt(2*g*h0_max+v0_max^2);
    v_min = -v_max;
    h_max = h0_max + 0.5 * v0_max^2 / g;
    x_range = [h_min, h_max; ...
               v_min, v_max];

    % Sample
    step_size = options.uniform_step_size;
    data = uniform_sampling_2_dim(x_range, step_size);
elseif sample_method == "trajectory"
    % Setup dynamics
    x_init_range = [0, 0.5; ...
                    -0.5, 0.5];
    u_range = [-10, 10];
    dyn_fun = @bouncing_ball_2_dim_dyn;

    % Sample
    num_steps = 100;
    step_size = options.traj_step_size;
    data = traj_sampling_2_dim(x_init_range, u_range, dyn_fun, num_steps, step_size = step_size);
    data = cell2mat(reshape(data, size(data, 2), 1));
elseif sample_method == "var_density"
    % Setup dynamical range
    h_min = 0; % must be zero
    h0_max = 0.5;
    v0_max = 0.5;
    v_max = sqrt(2*g*h0_max+v0_max^2);
    v_min = -v_max;
    h_max = h0_max + 0.5 * v0_max^2 / g;
    x_range = [h_min, h_max; ...
        v_min, v_max];

    % Sample
    min_step_size = options.var_density_min_step_size;
    max_step_size = options.var_density_max_step_size;
    data = var_density_sampling_bouncing_ball_2_dim(x_range, min_step_size, max_step_size);
end

% Calculate data_next
data_next = zeros(size(data));
parfor i = 1:size(data, 1)
    u = 0;
    x_next = dyn_fun(data(i, :)', u, g = g);
    data_next(i, :) = x_next';
end
end