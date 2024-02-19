function result = est_inner_product(g1_fun, g2_fun, state_next_points, state_DT)
% Estimate <gi(f(x)), gj(x)> numerically
N = size(state_DT.ConnectivityList, 1); % number of triangles
result = 0;
for i = 1:N
    % Get the indices of vertices of current triangle
    x1_idx = state_DT.ConnectivityList(i, 1);
    x2_idx = state_DT.ConnectivityList(i, 2);
    x3_idx = state_DT.ConnectivityList(i, 3);
    
    % Get the coordinate of vertices of current triangle
    x1 = state_DT.Points(x1_idx, :);
    x2 = state_DT.Points(x2_idx, :);
    x3 = state_DT.Points(x3_idx, :);
    
    % Calculate the average value of vertices
    G_avg = (g1_fun(state_next_points(x1_idx, :)') * conj(g2_fun(x1')) + ...
             g1_fun(state_next_points(x2_idx, :)') * conj(g2_fun(x2')) + ...
             g1_fun(state_next_points(x3_idx, :)') * conj(g2_fun(x3'))) / 3;
    
    % Calculate the area of the current triangle
    dv = cal_tri_area(x1, x2, x3);

    % Sum
    result = result + G_avg * dv;
end
end
