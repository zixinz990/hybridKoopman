function [data, data_extended, data_next, x_range] = gen_data(dyn_fun, sample_method, options)
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
    options.u_min(1, 1) {mustBeNumeric} = -10;
    options.u_max(1, 1) {mustBeNumeric} = 10;
    options.h0_max(1, 1) {mustBeNumeric, mustBePositive} = 0.5;
    options.v0_max(1, 1) {mustBeNumeric} = 0.5;
    options.h_lb_extend(1, 1) {mustBeNumeric, mustBePositive} = 0.15;
    options.h_ub_extend(1, 1) {mustBeNumeric, mustBePositive} = 0.15;
    options.v_lb_extend(1, 1) {mustBeNumeric, mustBePositive} = 0.5;
    options.v_ub_extend(1, 1) {mustBeNumeric, mustBePositive} = 0.5;
end

g = 10;

if sample_method == "uniform"
    % Setup dynamical range
    % extend the lower bound a bit, so that more observables can cover
    % boundaries    
    v_max = sqrt(2*g*options.h0_max+options.v0_max^2);
    v_min = -v_max;
    h_min = 0; % must be zero
    h_max = options.h0_max + 0.5 * options.v0_max^2 / g;
    x_range = [h_min, h_max; ...
               v_min, v_max];

    v_max_extend = v_max + options.v_ub_extend;
    v_min_extend = v_min - options.v_lb_extend;
    h_max_extend = h_max + options.h_ub_extend;
    h_min_extend = h_min - options.h_lb_extend;
    x_range_extend = [h_min_extend, h_max_extend; ...
                      v_min_extend, v_max_extend];

    % Sample
    step_size = options.uniform_step_size;
    data = uniform_sampling_2_dim(x_range, step_size);
    data_extended = uniform_sampling_2_dim(x_range_extend, step_size);
elseif sample_method == "trajectory"
    % Setup dynamics
    x_init_range = [0, 0.5; ...
        -0.5, 0.5];
    u_range = [-10, 10];
    dyn_fun = @bouncing_ball_2_dim_dyn;

    % Sample
    num_steps = 100;
    step_size = options.traj_step_size;
    data_extended = traj_sampling_2_dim(x_init_range, u_range, dyn_fun, num_steps, step_size = step_size);
    data_extended = cell2mat(reshape(data_extended, size(data_extended, 2), 1));
elseif sample_method == "var_density"
    % Setup dynamical range
    h_min = 0; % must be zero
    h_min = h_min - 0.15; % extend the lower bound a bit, so that more observables can cover the h=0 bound
    options.h0_max = 0.5;
    options.v0_max = 0.5;
    v_max = sqrt(2*g*options.h0_max+options.v0_max^2);
    v_min = -v_max;
    h_max = options.h0_max + 0.5 * options.v0_max^2 / g;
    x_range = [h_min, h_max; ...
        v_min, v_max];

    % Sample
    min_step_size = options.var_density_min_step_size;
    max_step_size = options.var_density_max_step_size;
    data_extended = var_density_sampling_bouncing_ball_2_dim(x_range, min_step_size, max_step_size);
end

% Remove all data that h = 0 and v <= 0
bad_x_init_idx = data(:, 1) == 0 & data(:, 2) <= 0;
data(bad_x_init_idx, :) = [];

% infeas_x_idx = data_extended(:, 1) <= 0;
% tmp = data_extended;
% data_extended(infeas_x_idx, :) = [];
% data = data_extended;
% data_extended = tmp;

% Generate list of control input
u_list = zeros(size(data, 1), 1);
for i = 1:size(u_list, 1)
    u_list(i, :) = rand * (options.u_max - options.u_min) + options.u_min;
end
data = [data, u_list];

% Calculate data_next
data_next = zeros(size(data));
for i = 1:size(data, 1)
    x_next = dyn_fun(data(i, 1:2)', u_list(i, :), g = g);
    data_next(i, :) = [x_next', u_list(i, :)];
end
end