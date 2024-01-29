function a = test_fun(b, options)
arguments
    b
    options.c double
    options.d double
    options.e {mustBeNonnegative} = 25.5
end
if b == 1
    a = options.c;
elseif b == 2
    a = options.d;
elseif b == 3
    a = options.e;
end
end