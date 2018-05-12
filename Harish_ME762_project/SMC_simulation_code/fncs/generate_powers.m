% Author: Davide Calzolari

function [ powers ] = generate_powers( n, x)
powers=zeros(1,n+1);
powers(1) = 1.0;
for i=2:1:(n+1);
    powers(i) = powers(i-1)*x;
end
end

