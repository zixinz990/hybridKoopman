function data = traj_sampling_2_dim(x_init_range, dynamics_discrete_time, num_steps, options)
% This function is used to sample trajectories over the state space
% Input:
%   x_init_range:           2 x 2 matrix. The 1st col is the lowerbound,
%                           the 2nd col is the upperbound
%   dynamics_discrete_time: function handle of the dynamics function, which
%                           takes col vectors and outputs col vectors
%   num_steps:              number of steps in each trajectory
% Output:
%   data: K x n matrix. K is the number of data, which is determined
%                       implicitly
arguments
    x_init_range(2, 2) {mustBeNumeric}
    dynamics_discrete_time
    num_steps(1, 1) {mustBeInteger, mustBePositive}
    options.x_init_density(1, 1) {mustBeNumeric, mustBePositive} = 0.01
end

x1_init_min = x_init_range(1, 1);
x2_init_min = x_init_range(2, 1);
x1_init_max = x_init_range(1, 2);
x2_init_max = x_init_range(2, 2);
x1_init_list = x1_init_min:options.x_init_density:x1_init_max;
x2_init_list = x2_init_min:options.x_init_density:x2_init_max;

% Generate all initial states
[X1_init, X2_init] = meshgrid(x1_init_list, x2_init_list);
x_init_list = [X1_init(:), X2_init(:)];

% Remove all initial states that h = 0 && v < 0
bad_x_init_idx = x_init_list(:, 1) == 0 & x_init_list(:, 2) < 0;
x_init_list(bad_x_init_idx, :) = [];

% Simulate
num_traj = size(x_init_list, 1);
traj_len = num_steps + 1;
data = zeros(traj_len*num_traj, 2);
for i = 1:num_traj
    cur_traj = zeros(traj_len, 2);
    cur_traj(1, :) = x_init_list(i, :);
    for k = 1:num_steps
        cur_traj(k+1, :) = dynamics_discrete_time(cur_traj(k, :)', 0)'; % don't consider control for now
    end
    data((i - 1)*traj_len+1:i*traj_len, :) = cur_traj;
end
end