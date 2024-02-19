function data = uniform_sampling_2_dim(x_range, step_size)
% Sampling uniformly over a 2-dim state space
% Input:
%   x_range:   2x2 matrix,
%              [x1_min, x1_max;
%               x2_min, x2_max]
%   step_size: 2x1 col vector, [x1_step_size; x2_step_size], which
%              determines the density of data
% Output:
%   data: nx2 matrix, n is the number of data, which is determined
%         implicitly by x_range and step_size
arguments
    x_range(2, 2) {mustBeNumeric}
    step_size(2, 1) {mustBeNumeric, mustBePositive}
end

% Dynamic range
x1_min = x_range(1, 1);
x2_min = x_range(2, 1);
x1_max = x_range(1, 2);
x2_max = x_range(2, 2);

% Step size
x1_step_size = step_size(1, 1);
x2_step_size = step_size(2, 1);

% Sample
[X1, X2] = meshgrid(x1_min:x1_step_size:x1_max, ...
                    x2_min:x2_step_size:x2_max);
data = unique([X1(:), X2(:)], 'rows');
end
