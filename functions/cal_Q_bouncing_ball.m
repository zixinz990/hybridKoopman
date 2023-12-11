function Q = cal_Q_bouncing_ball(g_list_xdot, g_f_list_xdot, xdot_lb, xdot_ub)
n_g = length(g_list_xdot); % number of observables

Q = zeros(n_g, n_g);
for i = 1:n_g
    for j = 1:n_g
        % integral g_i(f_k)*conj(g_j)
        fun = matlabFunction(g_f_list_xdot(i)*conj(g_list_xdot(j)));
        Q(i, j) = integral(fun, xdot_lb, xdot_ub);
    end
end
end
