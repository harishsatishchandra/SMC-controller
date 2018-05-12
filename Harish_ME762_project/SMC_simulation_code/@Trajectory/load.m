function load(obj,tau_file)

load(tau_file);
      
cx = create_poly_coefficients(gx);
cy = create_poly_coefficients(gy);

Xr = create_poly(cx);
dXr = create_poly(create_poly_derivative(cx,1));
ddXr = create_poly(create_poly_derivative(cx,2));

Yr = create_poly(cy);
dYr = create_poly(create_poly_derivative(cy,1));
ddYr = create_poly(create_poly_derivative(cy,2));

[S,W] = obj.arcLength(Xr,Yr);
%set the endpoint constraint to arc-length
row = find(gs(:,1)==0 & gs(:,2)>0,1);
gs(row,3) = S(1);
obj.T = gs(row,2); %end-time

cs = create_poly_coefficients(gs);
s = create_poly(cs);
v = create_poly(create_poly_derivative(cs,1));
a = create_poly(create_poly_derivative(cs,2));
j = create_poly(create_poly_derivative(cs,3));

%% time transform
theta_r = @(r)atan2(dYr(r),dXr(r));
v_r = @(r)sqrt(dXr(r).^2+dYr(r).^2);
dtheta_r = @(r)(dXr(r).*ddYr(r)-dYr(r).*ddXr(r))...
                   ./(dXr(r).^2+dYr(r).^2);

%time parametrized path
theta = @(t)theta_r(W(s(t)));
kappa = @(t)dtheta_r(W(s(t)))./v_r(W(s(t)));
dtheta = @(t)kappa(t).*v(t);

obj.id = tau_file;

obj.s = s;
obj.v = v;
obj.a = a;
obj.j = j;
obj.theta = theta;
obj.kappa = kappa;
obj.dtheta = dtheta;
obj.X = @(t)Xr(W(s(t)));
obj.Y = @(t)Yr(W(s(t)));
obj.dX = @(t)v(t).*cos(theta(t));
obj.dY = @(t)v(t).*sin(theta(t));
obj.ddX = @(t)a(t).*cos(theta(t)) - v(t).^2.*kappa(t).*sin(theta(t));
obj.ddY = @(t)a(t).*sin(theta(t)) + v(t).^2.*kappa(t).*cos(theta(t));

obj.ddtheta = @(t)numerical_derivative(dtheta,t);
obj.dddtheta = @(t)numerical_derivative2(dtheta,t);
