function data = var_density_sample_bouncing_ball_2_dim(x_range, min_step_len, max_step_len)
% This function is used to sample over the state space of the bouncing ball system [height; velocity]
% The sample density is changing uniformly over the space
% The density is higher when height -> 0 and velocity decreases
% Input:
%   x_range:      2 x 2 matrix. The 1st col is the lowerbound, the 2nd col is the upperbound
%   min_step_len: the minumum length between data points
%   n_data:       the number of data points
% Output:
%   data: n_data x 2 matrix
arguments
    x_range (2, 2) {mustBeNumeric}
    min_step_len (1, 1) {mustBeNumeric}
    max_step_len (1, 1) {mustBeNumeric}
end
x1_min = x_range(1, 1);
x2_min = x_range(2, 1);
x1_max = x_range(1, 2);
x2_max = x_range(2, 2);

n1 = round(2 * (x1_max - x1_min) / (max_step_len + min_step_len) + 1);
n2 = round(2 * (x2_max - x2_min) / (max_step_len + min_step_len) + 1);

delta_step_len_1 = (max_step_len - min_step_len) / (n1 - 2);
delta_step_len_2 = (max_step_len - min_step_len) / (n2 - 2);

x1_list = zeros(n1, 1);
x2_list = zeros(n2, 1);
x1_list(1) = x1_min;
x2_list(1) = x2_min;

for i = 1:n1 - 1
    x1_list(i+1) = x1_list(i) + min_step_len + (i-1) * delta_step_len_1;
end

for i = 1:n2 - 1
    x2_list(i+1) = x2_list(i) + min_step_len + (i-1) * delta_step_len_2;
end

[H, V] = meshgrid(x1_list, x2_list);
data = [H(:), V(:)];