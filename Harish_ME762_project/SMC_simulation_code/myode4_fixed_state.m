function [T,X,intState] = myode4_fixed_state(f,Tint,x0,intState0)

X = [x0';zeros(length(Tint)-1,length(x0))];
T = Tint;

intState=intState0;%;zeros(length(Tint)-1,length(intState0))];

for i = 1:length(T)-1
    j = i+1;
    ti = T(i);
    tj = T(j);
    h = tj-ti;
    
    
    xi = X(i,:)';
    
    
    [k1,intState{j}] = f( ti, xi,1,intState{j-1});
    k2 = f( ti + h/2, xi + h/2 * k1,0,intState{j});
    k3 = f( ti + h/2, xi + h/2 * k2,0,intState{j});
    k4 = f( ti + h, xi + h * k3,0,intState{j});    
    
    %R-K-4
    xj_4 = xi + h * (k1 + 2*k2 + 2*k3 + k4)/6;
    X(j,:) = xj_4';
end
