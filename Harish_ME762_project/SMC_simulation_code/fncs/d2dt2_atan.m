function [ derivative ] = d2dt2_atan( y,x,dy,dx,ddy,ddx )
%d2dt2_atan 
derivative=-(y*ddx)/(x^2+y^2)+(y*dx*(2*x*dx+2*y*dy))/(x^2+y^2)^2 ...
        -(x*dy*(2*x*dx+2*y*dy))/(x^2+y^2)^2+(x*ddy)/(x^2+y^2);
end

