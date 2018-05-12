function dxi = internalDynamics_vehicle_A(xi,p,dX,ddX,dY,ddY)
%interal dynamics for a vehicle breaking at front and rear axle with a static balance 
psi = xi(1);
omega = xi(2);
dxi = zeros(size(xi));
%decode_p_ctrlrA;

m=p.m;
R=p.R;
C_R=p.C_R;
B_R=p.B_R;
L=p.L;
l_R=p.l_R;
l_F=p.l_F;
h=p.h;
mu0=p.mu0;
J=p.J;
g=9.81;

balance = 1;

Ripsi = [cos(psi),sin(psi);-sin(psi),cos(psi)];
v = Ripsi*[dX;dY];
a = Ripsi*[ddX;ddY];
Fb = a(1) * m;

%velocity at rear wheel
vr = [v(1); v(2) - l_R * omega];

%rear normal tire force
Fzf = l_R/L * m*g - h/L * Fb;
Fzr = m*g - Fzf;
Fmaxr = mu0*Fzr;


%rear longitudinal tire force
Fxr = Fb * (1-balance);

%rear lateral tire force
fxy = @(omega_R)-Fmaxr*sin(C_R*atan(B_R*norm((vr-[R*omega_R;0])./norm(vr))/mu0))...
                        .*(vr-[R*omega_R;0])./norm(vr)/norm((vr-[R*omega_R;0])./norm(vr));
% fx = @(omega_R)[1,0]*fxy(omega_R);
% if Fxr>0
%     omega_R0 = v(1)/R+0.001;
% else
%     omega_R0 = v(1)/R-0.001;
% end
% if Fxr==0
    omega_R = v(1)/R;
% else
%     try
%         omega_R = fminsearch(@(omega)(fx(omega)-Fxr).^2,omega_R0);
%     catch 
%         asdf = 0;
%     end
% end

% vr_abs = norm(vr);
% sr = [0;vr(2)]/vr_abs;
% fyr = -sin(C_R*atan(B_R*sr(2)/mu0));
% Fyr = fyr * Fz0_R * mu0;
if norm(vr-[R*omega_R;0])==0
    Fyr = 0;
else
    Fyr = [0,1]*fxy(omega_R);
end

%rotational acceleration
domega = l_F*m/J * a(2) - L/J * Fyr;
dxi(1) = omega;
dxi(2) = domega;