function data = traj_sampling_2_dim(x_init_range, u_range, dyn_fun, num_steps, options)
% Sample several trajectories of a 2-dim dynamical system
% Input:
%   x_init_range:      2x2 matrix,
%                      [x1_init_min, x1_init_max;
%                       x2_init_min, x2_init_max]
%   u_range:           1x2 row vector, [u_min, u_max]
%   dyn_fun:           discrete dynamics function
%   num_steps:         length of each trajectory
%   options.step_size: 2x1 col vector, [x1_step_size; x2_step_size], which
%                      determines the density of data
% Output:
%   data: cell of trajectories, each element is a (num_steps+1)x2 matrix
%         which represents a trajectory
arguments
    x_init_range(2, 2) {mustBeNumeric}
    u_range(1, 2) {mustBeNumeric}
    dyn_fun
    num_steps(1, 1) {mustBeInteger, mustBePositive}
    options.step_size(2, 1) {mustBeNumeric, mustBePositive} = [0.01; 0.01]
end

% Dynamic range
x1_init_min = x_init_range(1, 1);
x2_init_min = x_init_range(2, 1);
x1_init_max = x_init_range(1, 2);
x2_init_max = x_init_range(2, 2);

% Control input range
u_min = u_range(1, 1);
u_max = u_range(1, 2);

% Step size
x1_step_size = options.step_size(1, 1);
x2_step_size = options.step_size(2, 1);

% Generate all initial states
[X1_init, X2_init] = meshgrid(x1_init_min:x1_step_size:x1_init_max, ...
                              x2_init_min:x2_step_size:x2_init_max);
x_init_list = [X1_init(:), X2_init(:)]; % nx2 matrix

% Remove all initial states that h = 0 && v <= 0
bad_x_init_idx = x_init_list(:, 1) == 0 & x_init_list(:, 2) <= 0;
x_init_list(bad_x_init_idx, :) = [];

% Initialize variables
num_traj = size(x_init_list, 1);
traj_len = num_steps + 1;
data = cell(1, num_traj);

% Simulate
parfor i = 1:num_traj
    cur_traj = zeros(traj_len, 2); % tall matrix
    cur_traj(1, :) = x_init_list(i, :);
    for k = 1:num_steps
        cur_traj(k+1, :) = dyn_fun(cur_traj(k, :)', 0)';
    end
    data{i} = cur_traj;
end
end
