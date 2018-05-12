function objD = transform(obj,lambda)
%% transforms the trajectory for the trajectory of a reference position

XR = @(t)obj.X(t) + cos(obj.psi0(t)).*lambda;
YR = @(t)obj.Y(t) + sin(obj.psi0(t)).*lambda;
dXR = @(t)obj.dX(t) - obj.dpsi0(t).*sin(obj.psi0(t)).*lambda;
dYR = @(t)obj.dY(t) + obj.dpsi0(t).*cos(obj.psi0(t)).*lambda;
ddXR = @(t)obj.ddX(t) - obj.ddpsi0(t).*sin(obj.psi0(t)).*lambda - obj.dpsi0(t).^2.*cos(obj.psi0(t)).*lambda;
ddYR = @(t)obj.ddY(t) + obj.ddpsi0(t).*cos(obj.psi0(t)).*lambda - obj.dpsi0(t).^2.*sin(obj.psi0(t)).*lambda;

vR = @(t)sqrt(dXR(t).^2+dYR(t).^2);
dvR = @(t)(dXR(t).*ddXR(t)+dYR(t).*ddYR(t))./vR(t);
thetaR = @(t)atan2(dYR(t),dXR(t));
dthetaR = @(t)(dXR(t).*ddYR(t)-dYR(t).*ddXR(t))./vR(t).^2;


TS = 0:0.001:obj.T;
TS2 = TS;
TS2(1) = TS(1) + 1e-5;
TS2(end) = TS(end) - 1e-5;

%numerisch
ddthetaS = numerical_derivative(dthetaR,TS2);
dddthetaS = numerical_derivative2(dthetaR,TS2);
ddvS = numerical_derivative(dvR,TS2);

%interpolation
ddthetaR = @(t)interp1(TS,ddthetaS,t);
dddthetaR = @(t)interp1(TS,dddthetaS,t);
ddvR = @(t)interp1(TS,ddvS,t);


objD = Trajectory();
objD.T = obj.T;
objD.lambda = lambda;
objD.X = XR;
objD.dX = dXR;
objD.ddX = ddXR;
objD.Y = YR;
objD.dY = dYR;
objD.ddY = ddYR;
objD.s = @(t)0; %not defined (not used)
objD.v = vR;
objD.a = dvR;
objD.j = ddvR;
objD.kappa = @(t)0; %not defined (not used)
objD.theta = thetaR;
objD.dtheta = dthetaR;
objD.ddtheta = ddthetaR;
objD.dddtheta = dddthetaR;
% objD.psi0 = @(t)obj.psi0(t);
% objD.dpsi0 = @(t)obj.dpsi0(t);
% objD.ddpsi0 = @(t)obj.ddpsi0(t);
