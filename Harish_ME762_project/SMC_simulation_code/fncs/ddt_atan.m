function [ derivative ] = ddt_atan( y,x,dy,dx )
%ATAN_DERIVATIVE 
if y^2+x^2==0
    derivative=0;
    return
end
derivative=(y*dx-x*dy)/(y^2+x^2);
end

