function [g, rbf_center_list] = gen_gaussian_rbfs_2_dim(num_rbf, eps, data)
% Generate a list of 2-dim Gaussian RBF observables given some data points
% Gaussian RBF has the form: g(x) = exp(-eps^2 * |x - c|^2)
% Input:
%   num_obs: number of observables
%   eps:     parameter of Gaussian RBF
%   data:    some points in the state space
% Output:
%   g:               1xnum_obs cell, each element is an observable function
%                    handle
%   rbf_center_list: num_obsx2 matrix, each row is the center of a RBF
arguments
    num_rbf(1,1) {mustBeNumeric, mustBeInteger, mustBePositive}
    eps(1,1) {mustBeNumeric, mustBePositive}
    data {mustBeNumeric}
end

syms x1 x2 real
x = [x1; x2];

% Find the center of RBFs using kmeans++
[~, rbf_center_list] = kmeans(data, num_rbf);

% Create g_list
g = cell(1, num_rbf);
for i = 1:num_rbf
    rbf_center_i = rbf_center_list(i, :)';
    gi = exp(-eps^2 * (x - rbf_center_i)' * (x - rbf_center_i));
    g{i} = matlabFunction(gi, 'Vars', {x});
end

end
