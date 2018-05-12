function [dx,y] = vmodel_A(t,x,u,p)
%Bicycle Model with 
% - normal force equilibrium for pitching-moments
% - nonlinear tyre model
% state x=[X,Y,psi,vx,vy,omega]
% input u=[delta,omega_f,omega_r]

%get state and parameters from x, p vectors
m=p.m;
J=p.J;
l_F=p.l_F;
l_R=p.l_R;
L=p.L;
h=p.h;
mu0=p.mu0;
B_F=p.B_F;
C_F=p.C_F;
B_R=p.B_R;
C_R=p.C_R;
R=p.R;
g=9.81;

%state
    %position
    X = x(1);
    Y = x(2);
    psi = x(3);
    
    %velocity
    vx = x(4);
    vy = x(5);
    omega = x(6);

%input (with input limitations)
delta = max(-pi/4,min(u(1),pi/4));
vf = [vx;vy+l_F*omega]; %velocity at wheel center front
vfW = [cos(delta),sin(delta);-sin(delta),cos(delta)] * vf; % in wheel coordinates
omega_F = max(min(vfW(1)/R,0),min(u(2),max(vfW(1)/R,0)));
vr = [vx;vy-l_R*omega]; %velocity at wheel center rear
omega_R = max(min(vr(1)/R,0),min(u(3),max(vr(1)/R,0)));

%FRONT
vf_abs = norm(vf);
vfL = [-cos(delta);-sin(delta)]*omega_F*R; %velocity of Latsch
vfL_abs = norm(vfL);
sf = (vf+vfL)/max(vf_abs,vfL_abs); %slip vector
sf_abs = norm(sf);
ff_abs = sin(C_F*atan(B_F*sf_abs/mu0));
if sf_abs==0
    ff = zeros(2,1);
else
    ff  =ff_abs * -sf/sf_abs;
end

%REAR
vr_abs = norm(vr);
vrL = [-1;0]*omega_R*R; %velocity of Latsch
vrL_abs = norm(vrL);
sr = (vr+vrL)/max(vr_abs,vrL_abs); %slip vector
sr_abs = norm(sr);
fr_abs = sin(C_R*atan(B_R*sr_abs/mu0));
if sr_abs==0
    fr = zeros(2,1);
else
    fr  =fr_abs * -sr/sr_abs;
end

%calculate normal forces
Fzf = l_R*m*g /(L+ff(1)*mu0*h);
Fzr = m*g - Fzf;

%FRONT FORCES
Ff = ff * mu0 * Fzf;

%REAR FORCES
Fr = fr * mu0 * Fzr;

%ACCELERATIONS
dv = [];
dv(1,1) = (Ff(1)+Fr(1))/m + vy * omega;
dv(2,1) = (Ff(2)+Fr(2))/m - vx * omega;
domega = (l_F * Ff(2) - l_R * Fr(2))/J;

%dx = zeros(size(x));
%position
    cp = cos(psi);sp = sin(psi);
    dx(1,1) = cp * vx - sp * vy;
    dx(2,1) = sp * vx + cp * vy;
    dx(3,1) = omega;
%velocity
    dx(4,1) = dv(1);
    dx(5,1) = dv(2);
    dx(6,1) = domega;
    dx(7,1) = u(4);
    
%disp(['AX: ' num2str(dv(1))]);
    
y = [ff;fr;delta;Ff;Fr];