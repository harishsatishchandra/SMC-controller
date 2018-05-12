function pd = create_poly_derivative(p,n)
pd = p;
if n>0
    if length(pd)>1
        pd = pd(2:end);
        for i=1:length(pd)
            pd(i) = pd(i) * i;
        end
        pd = create_poly_derivative(pd,n-1);
    else
        pd(1) = 0;
    end
end