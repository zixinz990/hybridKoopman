function results = inner_product(f1, f2, a, b)
% f1s, f2s are vectors of symbolic functions in a Hilbert space defined in [a, b]

% Use arrayfun to compute the inner product for each pair of functions
results = arrayfun(@(f1, f2) int(f1*conj(f2), a, b), f1, f2);

end
