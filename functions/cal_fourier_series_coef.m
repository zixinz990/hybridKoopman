function [a0, an, bn] = cal_fourier_series_coef(f, x, l, N)
% Calculate the Fourier series coefficients of a scalar function
% Input:
%   f: function handle of a scalar function
%   l: f is defined on [-l, l]
%   N: order of f
% Output:
%   a0: the term outside the series
%   an: coefficients of cos term
%   bn: coefficients of sin term

a0 = integral(f, -l, l) / (2*l);

an = zeros(N, 1);
bn = zeros(N, 1);

for n = 1:N
    fun_1 = matlabFunction(f * cos(n*pi*x/l));
    fun_2 = matlabFunction(f * sin(n*pi*x/l));
    an(n) = integral(fun_1, -l, l) / l;
    bn(n) = integral(fun_2, -l, l) / l;
end
end