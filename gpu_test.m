clear; clc;

Ga = rand(1000, 'single');
% Ga = rand(1000, 'single', 'gpuArray');
start_time = cputime;
for n = 1:50    
    parfor i = 1:1000
        Gfft = fft(Ga);
        Gb = (real(Gfft) + Ga) * i;
    end    
end
end_time = cputime;
total_time = end_time - start_time
