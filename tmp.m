clear;
clc;
addpath(genpath("./functions"));
addpath("./data");

%% Sample data
u_min = -10;
u_max = 20;
dyn_fun = @bouncing_ball_2_dim_dyn;
sample_method = "uniform";
uniform_step_size = [0.015; 0.015];
h_lb_extend = 0.2;
h_ub_extend = 0.2;
v_lb_extend = 0.2;
v_ub_extend = 0.2;
% x_range = [h_min, h_max; ...
%            v_min, v_max];
[data, data_extended, data_next, x_range] = gen_data(dyn_fun, sample_method, ...
    uniform_step_size = uniform_step_size, ...
    u_min = u_min, u_max = u_max, ...
    h_lb_extend = h_lb_extend, h_ub_extend = h_ub_extend, ...
    v_lb_extend = v_lb_extend, v_ub_extend = v_ub_extend);
h_min = x_range(1, 1);
h_max = x_range(1, 2);
v_min = x_range(2, 1);
v_max = x_range(2, 2);
x_range_extend = x_range + [-h_lb_extend, h_ub_extend; -v_lb_extend, v_ub_extend];
fprintf("Number of data: %d\n", size(data, 1));

data_DT = delaunayTriangulation(data(:, 1:2)); % Delaunay triangulation

%% Generate two Gaussian RBF functions
syms x1 x2 real
x = [x1; x2];

eps = 1;

c1 = [0.5; 0.5]; % center 1
c2 = [0.5; 0.5]; % center 2

g1 = exp(-eps^2*(x - c1)'*(x - c1));
g2 = exp(-eps^2*(x - c2)'*(x - c2));
g1g2_fun = matlabFunction(g1 * conj(g2));

%% <g1, g2> in the x_range: true value
tic
inner_product_truth = integral2(g1g2_fun, h_min, h_max, v_min, v_max)
toc

%% <g1, g2> in the x_range: estimated value
g1_fun = matlabFunction(g1, 'Vars', {x});
g2_fun = matlabFunction(g2, 'Vars', {x});
tic
inner_product_est = est_inner_product(g1_fun, g2_fun, data, data_DT)
toc
