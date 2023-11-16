close all; clear; clc;

syms v real

f = v;
N = 50;
l = 0.5;
[a0, an, bn] = cal_fourier_series_coef(matlabFunction(f), v, l, N);

fourier_series = sym(a0);
for n = 1:10
    fourier_series = fourier_series ...
                     + an(n) * cos(n*pi*v/l) ...
                     + bn(n) * sin(n*pi*v/l);
end

v_ = -l:0.01:l;
result = subs(fourier_series, v, v_);

plot(v_, result), hold on;
plot(v_, subs(f, v, v_));
axis equal, grid on;
