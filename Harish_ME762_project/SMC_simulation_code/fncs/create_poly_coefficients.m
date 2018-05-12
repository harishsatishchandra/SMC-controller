function p = create_poly_coefficients(g)
n = size(g,1);
assert(size(g,2)==3);

% degree n-1
% coefficients p
% list with n constraints g

% create factors of derivatives
C = {ones(1,n)};
for i=1:n-1
    c = C{i};
    c = c(2:end);
    m = length(c);
    for j = 1:m
        c(j) = c(j).* j;
    end
    C{end+1} = c; %#ok<AGROW>
end

% create matrix with contraints
b = g(:,3);    %constraint value
A = zeros(n,n);
for i=1:n;
    m = g(i,1);%derivative order of constraint
    a = C{m+1};%corresponding derivative factors
    t = g(i,2);%time of constraint
    for j = 1:length(a)
        a(j) = a(j) .* (t.^(j-1));
    end
    A(i,(n-length(a)+1):n) = a;
end
% constraints: A*p = b
p = A\b;