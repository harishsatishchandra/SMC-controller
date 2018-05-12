function [ invT, C] = invT_bicycle( b,steer,y,p)
%INVT_BICYCLE 
theta=y(3);
v=y(4);

invT=[cos(theta)/(cos(theta)^2 + sin(theta)^2),            sin(theta)/(cos(theta)^2 + sin(theta)^2)
 -(p.L*sin(theta) + b*cos(theta)*tan(steer))/(b*cos(theta)^2*v + b*sin(theta)^2*v + b*cos(theta)^2*tan(steer)^2*v + b*sin(theta)^2*tan(steer)^2*v), (p.L*cos(theta) - b*sin(theta)*tan(steer))/(b*cos(theta)^2*v + b*sin(theta)^2*v + b*cos(theta)^2*tan(steer)^2*v + b*sin(theta)^2*tan(steer)^2*v)];

C=[-(tan(steer)*v^2*(p.L*sin(theta) + b*cos(theta)*tan(steer)))/p.L^2
  (tan(steer)*v^2*(p.L*cos(theta) - b*sin(theta)*tan(steer)))/p.L^2];

end

