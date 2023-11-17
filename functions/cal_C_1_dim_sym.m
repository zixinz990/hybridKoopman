function C = cal_C_1_dim_sym(phi_list, g_list, x_lb, x_ub)
% To use this function, state x must be a 1-dim variable
% Input:
%   phi_list: an orhonormal set of basis functions
%   g_list: list of observables
% Output:
%   C: eq. 24 in the paper

n_phi = length(phi_list);
n_g = length(g_list);

C = zeros(n_g, n_phi);
for i = 1:n_g
    for j = 1:n_phi
        fun = g_list(i)*conj(phi_list(j));
        C(i, j) = int(fun, x_lb, x_ub);
    end
end
end