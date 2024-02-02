function data = uniform_sampling_2_dim(x_range, step_size)
% This function is used to sample uniformly over a state space
% Input:
%   x_range:  2 x 2 matrix. The 1st col is the lowerbound, the 2nd col is
%             the upperbound
%   step_len: the length between data points
% Output:
%   data: K x n matrix. K is the number of data, which is determined
%         implicitly by x_range and num_steps
arguments
    x_range(2, 2) {mustBeNumeric}
    step_size(1, 1) {mustBeNumeric, mustBePositive}
end

x1_min = x_range(1, 1);
x2_min = x_range(2, 1);
x1_max = x_range(1, 2);
x2_max = x_range(2, 2);
[H, V] = meshgrid(x1_min:step_size:x1_max, x2_min:step_size:x2_max);
data = unique([H(:), V(:)], 'rows');
end