function dy = numerical_derivative(f,x)
eps = 1e-8;
dy = (f(x+eps)-f(x-eps))./eps./2;
