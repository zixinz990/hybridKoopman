function R = cal_R_1_dim(g_list, x_lb, x_ub)
% To use this function, state x must be a 1-dim variable
% Input:
%   g_list: observables, an independent and complete set of basis functions
% Output:
%   R: eq. 33 in the paper

n_g = length(g_list);

R = zeros(n_g, n_g);
if g_list(1) == 1
    R(1, 1) = x_ub - x_lb;
    for i = 1:n_g
        for j = 1:n_g
            if ~(i == 1 && j == 1)
                fun = matlabFunction(g_list(i)*conj(g_list(j)));
                R(i, j) = integral(fun, x_lb, x_ub);
            end
        end
    end
else
    for i = 1:n_g
        for j = 1:n_g
            fun = matlabFunction(g_list(i)*conj(g_list(j)));
            R(i, j) = integral(fun, x_lb, x_ub);
        end
    end
end
end