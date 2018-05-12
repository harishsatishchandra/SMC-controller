function f = create_poly(p)
%creates polynomial function for coefficients in p
%p is ordered by increasing exponent
f = @(x)poly(x,p);

function y = poly(x,p)
y = ones(size(x)).*p(end);
for i=1:length(p)-1
    y = y .* x;
    y = y + p(end-i);
end