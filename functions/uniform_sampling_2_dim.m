function state_data = uniform_sampling_2_dim(x_range, step_len)
% This function is used to sample data uniformly in a set of states
% Input:
%   x_range:  2 x 2 matrix. The 1st col is the
%             min value, the 2nd col is the max value
%   step_len: the length between data points
% Output:
%   state_points: K x n matrix. K is the number of data
x1_min = x_range(1, 1);
x2_min = x_range(2, 1);
x1_max = x_range(1, 2);
x2_max = x_range(2, 2);
[H, V] = meshgrid(x1_min:step_len:x1_max, x2_min:step_len:x2_max);
state_data = unique([H(:), V(:)]);
end