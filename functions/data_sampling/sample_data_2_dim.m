function state_data = sample_data_2_dim(x_range, sample_method, options)
% This function is used to sample data in a set of states
% Input:
%   x_range:       n x 2 matrix. n is the dim of the state. The 1st col is t
%                  the min value, the 2nd col is the max value
%   sample_method: method of sampling. It can be: uniform, ...
% Output:
%   state_data: K x n matrix. K is the number of data
arguments
    x_range (2, 2) {mustBeNumeric}
    sample_method (1, 1) string = "uniform"
    options.step_len (1, 1) {mustBeNonnegative} = 0.01
end
if sample_method == "uniform"
    state_data = uniform_sampling_2_dim(x_range, options.step_len);
else
end