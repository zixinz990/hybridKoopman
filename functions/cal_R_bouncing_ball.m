function R = cal_R_bouncing_ball(g_list_xdot, xdot_lb, xdot_ub)
% To use this function, state x must be a 2-dim variable
% Input:
%   g_list: observables, an independent and complete set of basis functions
% Output:
%   R: eq. 33 in the paper

n_g = length(g_list_xdot);

R = zeros(n_g, n_g);
for i = 1:n_g
    for j = 1:n_g
        fun = matlabFunction(g_list_xdot(i)*conj(g_list_xdot(j)));
        R(i, j) = integral(fun, xdot_lb, xdot_ub);
    end
end
end
