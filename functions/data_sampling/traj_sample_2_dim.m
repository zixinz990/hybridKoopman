function data = traj_sample_2_dim(init_state_range, dynamics, sim_step)
% This function is used to sample trajectories over the state space
% Input:
%   init_state_range:  2 x 2 matrix. The 1st col is the lowerbound, the 2nd col is the upperbound
%   dynamics:          function handle of the dynamics function, which takes col vectors and outputs col vectors
%   sim_step:          number of steps in each trajectory
% Output:
%   data: K x n matrix. K is the number of data, which is determined implicitly
x1_init_min = init_state_range(1, 1);
x2_init_min = init_state_range(2, 1);
x1_init_max = init_state_range(1, 2);
x2_init_max = init_state_range(2, 2);

x1_init_list = x1_init_min:0.1:x1_init_max;
x2_init_list = x2_init_min:0.1:x2_init_max;

% generate all initial state
[X1_init, X2_init] = meshgrid(x1_init_list, x2_init_list);
x_init_list = [X1_init(:), X2_init(:)];

bad_x_init_idx = x_init_list(:, 1) == 0 & x_init_list(:, 2) < 0;
x_init_list(bad_x_init_idx, :) = [];

num_traj = size(x_init_list, 1);
traj_len = sim_step + 1;
data = zeros(traj_len*num_traj, 2);

% dynamics roll out
for i = 1:num_traj
    cur_traj = zeros(traj_len, 2);
    cur_traj(1, :) = x_init_list(i, :);
    for k = 1:sim_step
        cur_traj(k+1, :) = dynamics(cur_traj(k, :)', 0)'; % don't consider control for now
    end
    data((i - 1)*traj_len+1:i*traj_len, :) = cur_traj;
end
end