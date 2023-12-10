function Q = cal_Q_1_dim(g_list, f_list, f_range)
% To use this function, state x must be a 1-dim variable
% Input:
%   g_list: observables, an independent and complete set of basis functions
%   f_list: list of dynamics for different mode, may be hybrid
%   f_range: row i is the range of x for fi
% Output:
%   Q: eq. 29 in the paper

n_g = length(g_list); % number of observables
n_f = length(f_list); % number of dynamics

% compose g and f
g_f_list = sym(zeros(n_g, n_f)); % list of g(f(x))
for i = 1:n_g
    for j = 1:n_f
        g_f_list(i, j) = subs(g_list(i), f_list(j));
    end
end

Q = zeros(n_g, n_g);
if g_list(1) == 1
    for k = 1:n_f
        Q(1, 1) = Q(1, 1) + f_range(k, 2) - f_range(k, 1);
    end
    for i = 1:n_g
        for j = 1:n_g
            disp(i+","+j)
            if ~(i == 1 && j == 1)
                % integral phi_i(f_1)*conj(phi_j) + ... + phi_i(f_n_f)*conj(phi_j)
                for k = 1:n_f
                    % integral phi_i(f_k)*conj(phi_j)
                    fun = matlabFunction(g_f_list(i, k)*conj(g_list(j)));
                    Q(i, j) = Q(i, j) + integral(fun, f_range(k, 1), f_range(k, 2));
                end
            end
        end
    end
else
    for i = 1:n_g
        for j = 1:n_g
            disp(i+","+j)
            Q(i, j) = 0;
            % integral phi_i(f_1)*conj(phi_j) + ... + phi_i(f_n_f)*conj(phi_j)
            for k = 1:n_f
                % integral phi_i(f_k)*conj(phi_j)
                fun = matlabFunction(g_f_list(i, k)*conj(g_list(j)));
                Q(i, j) = Q(i, j) + integral(fun, f_range(k, 1), f_range(k, 2));
            end
        end
    end
end
end