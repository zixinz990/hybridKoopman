function C = cal_C_1_dim(phi_list, g_list, x_lb, x_ub)
% To use this function, state x must be a 1-dim variable
% Input:
%   phi_list: an orhonormal set of basis functions
%   g_list: list of observables
% Output:
%   C: eq. 24 in the paper

n_phi = length(phi_list);
n_g = length(g_list);
C = zeros(n_g, n_phi);

if phi_list(1) == 1 && g_list(1) == 1
    C(1, 1) = x_ub - x_lb;
    for i = 1:n_g
        for j = 1:n_phi
            if ~(i == 1 && j == 1)
                fun = matlabFunction(g_list(i)*conj(phi_list(j)));
                C(i, j) = integral(fun, x_lb, x_ub);
            end
        end
    end
else
    for i = 1:n_g
        for j = 1:n_phi
            fun = matlabFunction(g_list(i)*conj(phi_list(j)));
            C(i, j) = integral(fun, x_lb, x_ub);
        end
    end
end
end