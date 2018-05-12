function dy = numerical_derivative2(f,x)
eps = 1e-6;
dy = ( f(x+2*eps) - 2.*f(x) + f(x-2*eps) ) ./ (4*eps^2);
