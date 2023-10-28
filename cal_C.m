function C = cal_C(m)
% m is the number of observables
C = zeros(m, m);
C(1, 1) = 1;
for n = 1:(m-1)/2
    C(2*n, 2*n) = 0.5;
    C(2*n, 2*n+1) = 0.5;
    C(2*n+1, 2*n) = -0.5i;
    C(2*n+1, 2*n+1) = 0.5i;
end
end