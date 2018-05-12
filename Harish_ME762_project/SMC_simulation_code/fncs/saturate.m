function [ saturated_x ] = saturate( x,lb,ub)
%SATURATE
saturated_x = min(ub, max(lb, x));
end

