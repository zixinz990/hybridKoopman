function data = var_density_sampling_bouncing_ball_2_dim(x_range, min_step_size, max_step_size)
% Sample over a 2-dim state space
% The sampling density is changing over the space
% The density is higher when h -> 0 and v decreases
% Input:
%   x_range:       2x2 matrix,
%                  [x1_min, x1_max;
%                   x2_min, x2_max]
%   min_step_size: 2x1 col vector, [x1_min_step_size; x2_min_step_size],
%                  which determines the maximum density of data
%   max_step_size: 2x1 col vector, [x1_max_step_size; x2_max_step_size],
%                  which determines the minimum density of data
% Output:
%   data: nx2 matrix, n is the number of data, which is determined
%         implicitly by x_range, min_step_size and max_step_size
arguments
    x_range (2, 2) {mustBeNumeric}
    min_step_size (2, 1) {mustBeNumeric}
    max_step_size (2, 1) {mustBeNumeric}
end

% Dynamic range
x1_min = x_range(1, 1);
x2_min = x_range(2, 1);
x1_max = x_range(1, 2);
x2_max = x_range(2, 2);

% Calculate number of x1 and x2
n1 = round(2 * (x1_max - x1_min) / (max_step_size(1) + min_step_size(1)) + 1);
n2 = round(2 * (x2_max - x2_min) / (max_step_size(2) + min_step_size(2)) + 1);

% Calculate the change of density
delta_step_len_1 = (max_step_size(1) - min_step_size(1)) / (n1 - 2);
delta_step_len_2 = (max_step_size(2) - min_step_size(2)) / (n2 - 2);

% Initialize variables
x1_list = zeros(n1, 1);
x2_list = zeros(n2, 1);
x1_list(1) = x1_min;
x2_list(1) = x2_min;

% Calculate value of x1 and x2
for i = 1:n1 - 1
    x1_list(i+1) = x1_list(i) + min_step_size(1) + (i-1) * delta_step_len_1;
end

for i = 1:n2 - 1
    x2_list(i+1) = x2_list(i) + min_step_size(2) + (i-1) * delta_step_len_2;
end

% Generate data matrix
[H, V] = meshgrid(x1_list, x2_list);
data = [H(:), V(:)];
end
