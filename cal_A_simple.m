function A_simple = cal_A_simple(m)
% phi_list is a set of orthonormal basis functions
% F1, F2 are functions of hybrid dynamics
% x is a symbolic var
% x_star is the jump point
% m is the number of observables

syms s t real

A_simple_element = ...
    - (exp((pi*s*48i)/25)*1i)/(2*pi*(3*s + t)) + (exp(-(pi*t*16i)/25)*1i)/(2*pi*(3*s + t)) - (25*exp((pi*s*31i)/50)*exp(-pi*t*2i)*(exp((pi*s*799i)/625)*1i - exp((pi*t*34i)/25)*1i))/(pi*(47*s - 50*t));

A_simple_element_1 = ...
    (8*exp((pi*s*48i)/25))/25 + (25*(exp((pi*s*127i)/50)*1i - exp((pi*s*9873i)/1250)*1i))/(197*s*pi);

A_simple_element_2 = ...
    (17*exp((pi*s*23i)/1250))/25 - (25*exp(-(pi*s*376i)/625)*(exp((pi*s*1576i)/625)*1i - 1i))/(197*s*pi);

A_simple = zeros(m, m);
for u = 1:m
    for v = 1:m
        s_ = (-1)^u*floor(u/2);
        t_ = (-1)^v*floor(v/2);
        if (u==1 && v==1)
            A_simple(1, 1) = 1;
        elseif (-3*s_-t_==0)
            A_simple(u, v) = double(subs(A_simple_element_1, [s; t], [s_; t_]));
        elseif (s_*0.94-t_==0)
            A_simple(u, v) = double(subs(A_simple_element_2, [s; t], [s_; t_]));
        else
            A_simple(u, v) = double(subs(A_simple_element, [s; t], [s_; t_]));
        end
    end
end        
end