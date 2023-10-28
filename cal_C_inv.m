function C_inv = cal_C_inv(m)
% m is the number of observables
C_inv = zeros(m, m);
C_inv(1, 1) = 1;
for n = 1:(m-1)/2
    C_inv(2*n, 2*n) = 1;
    C_inv(2*n+1, 2*n) = 1;
    C_inv(2*n, 2*n+1) = 1i;
    C_inv(2*n+1, 2*n+1) = -1i;
end
end