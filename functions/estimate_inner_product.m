function result = estimate_inner_product(g1_fun, g2_fun, f_fun, state_DT, dv)
% Estimate <gi(f(x)), gj(x)>
N = size(state_DT.ConnectivityList, 1); % number of triangles
result = 0;
for i = 1:N
    x1 = state_DT.Points(state_DT.ConnectivityList(i, 1), :);
    x2 = state_DT.Points(state_DT.ConnectivityList(i, 2), :);
    x3 = state_DT.Points(state_DT.ConnectivityList(i, 3), :);

    G_avg = (g1_fun(f_fun(x1', 0)) * conj(g2_fun(x1')) + ...
             g1_fun(f_fun(x2', 0)) * conj(g2_fun(x2')) + ...
             g1_fun(f_fun(x3', 0)) * conj(g2_fun(x3'))) / 3;
    result = result + G_avg * dv;
end
end
