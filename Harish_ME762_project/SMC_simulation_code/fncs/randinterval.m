function [ out ] = randinterval( x,dim)
%RANDINTERVAL random numbers in interval +/- x
out=-x + (x+x)*rand(dim,1);
end

