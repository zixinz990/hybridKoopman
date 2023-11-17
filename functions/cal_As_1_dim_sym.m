function As = cal_As_1_dim_sym(phi_list, f_list, f_range)
% To use this function, state x must be a 1-dim variable
% Input:
%   phi_list: an orhonormal set of basis functions
%   f_list: list of dynamics for different mode, may be hybrid
%   f_range: row i is the range of x for fi
% Output:
%   As: eq. 22 in the paper

n_phi = length(phi_list); % number of basis functions
n_f = length(f_list); % number of dynamics

% compose phi and f
phi_f_list = sym(zeros(n_phi, n_f)); % list of phi(f(x))
for i = 1:n_phi
    for j = 1:n_f
        phi_f_list(i, j) = subs(phi_list(i), f_list(j));
    end
end

As = sym(zeros(n_phi, n_phi));

for i = 1:n_phi
    for j = 1:n_phi
        As(i, j) = 0;
        % integral phi_i(f_1)*conj(phi_j) + ... + phi_i(f_n_f)*conj(phi_j)
        for k = 1:n_f
            % integral phi_i(f_k)*conj(phi_j)
            fun = phi_f_list(i, k)*conj(phi_list(j));
            As(i, j) = As(i, j) + int(fun, f_range(k, 1), f_range(k, 2));
        end
    end
end
end