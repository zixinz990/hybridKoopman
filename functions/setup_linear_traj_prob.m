function traj_opt_prob = setup_linear_traj_prob(N, x0, X_ref, ...
                                                u_min, u_max, ...
                                                A, B, C, ...
                                                Q, Q_terminal, R)
% Input:
%   N:          horizon length
%   x0:         initial state, nx1 vector
%   X_ref:      reference trajectory of states, nxN matrix
%   u_min:      lower bound of input, scalar
%   u_max:      upper bound of input, scalar
%   A & B & C:  linear dynamics: x_next = A*x + B*u + C
%   Q:          running cost state weight matrix
%   Q_terminal: terminal cost state weight matrix
%   R:          input weight matrix
% Output:
%   traj_opt_prob: a trajectory optimization problem
arguments
    N{mustBeNumeric}
    x0{mustBeNumeric}
    X_ref{mustBeNumeric}
    u_min(1, 1) {mustBeNumeric}
    u_max(1, 1) {mustBeNumeric}
    A{mustBeNumeric}
    B{mustBeNumeric}
    C{mustBeNumeric}
    Q{mustBeNumeric}
    Q_terminal{mustBeNumeric}
    R{mustBeNumeric}
end

% Initialize the problem and variables
traj_opt_prob = optimproblem;

x_dim = size(x0, 1); % state dimension
X = optimexpr(x_dim, N);
U = optimvar("U", N-1, "LowerBound", u_min, "UpperBound", u_max);

% Simulate the whole trajectory
X(:, 1) = x0;
for i = 1:N - 1
    X(:, i+1) = A * X(:, i) + B * U(i) + C;
end

% Running quadratic cost
cost = optimexpr(1);
for i = 1:N - 1
    cost = cost + (X(:, i) - X_ref(:, i))' * Q * (X(:, i) - X_ref(:, i)) + U(i)' * R * U(i);
end

% Terminal quadratic cost
cost = cost + (X(:, end) - X_ref(:, end))' * Q_terminal * (X(:, end) - X_ref(:, end));
traj_opt_prob.Objective = cost;

end
