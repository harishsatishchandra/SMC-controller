function obj_static = make_static(obj,supports)
TS = linspace(0,obj.T,supports);
TS2 = TS;
TS2(1) = TS2(1)+1e-5;
TS2(end) = TS2(end)-1e-5;

sS = obj.s(TS2);
vS = obj.v(TS2);
aS = obj.a(TS2);
jS = obj.j(TS2);
thetaS = obj.theta(TS2);
% psi0S = obj.psi0(TS2);
% dpsi0S = obj.dpsi0(TS2);
% ddpsi0S = obj.ddpsi0(TS2);
kappaS = obj.kappa(TS2);
dthetaS = obj.dtheta(TS2);
ddthetaS = obj.ddtheta(TS2);
dddthetaS = obj.dddtheta(TS2);
XS = obj.X(TS2);
YS = obj.Y(TS2);
dXS = obj.dX(TS2);
dYS = obj.dY(TS2);
ddXS = obj.ddX(TS2);
ddYS = obj.ddY(TS2);

obj_static = Trajectory();
obj_static.T = obj.T;
obj_static.s = @(t)myinterp(TS,sS,t);
obj_static.v = @(t)myinterp(TS,vS,t);
obj_static.a = @(t)myinterp(TS,aS,t);
obj_static.j = @(t)myinterp(TS,jS,t);
obj_static.theta = @(t)myinterp(TS,thetaS,t);
% obj_static.psi0 = @(t)myinterp(TS,psi0S,t);
% obj_static.dpsi0 = @(t)myinterp(TS,dpsi0S,t);
% obj_static.ddpsi0 = @(t)myinterp(TS,ddpsi0S,t);
obj_static.kappa = @(t)myinterp(TS,kappaS,t);
obj_static.dtheta = @(t)myinterp(TS,dthetaS,t);
obj_static.ddtheta = @(t)myinterp(TS,ddthetaS,t);
obj_static.dddtheta = @(t)myinterp(TS,dddthetaS,t);
obj_static.X = @(t)myinterp(TS,XS,t);
obj_static.dX = @(t)myinterp(TS,dXS,t);
obj_static.ddX = @(t)myinterp(TS,ddXS,t);
obj_static.Y = @(t)myinterp(TS,YS,t);
obj_static.dY = @(t)myinterp(TS,dYS,t);
obj_static.ddY = @(t)myinterp(TS,ddYS,t);

function x = myinterp(t,x,ti)
x = interp1(t,x,ti,'spline');









