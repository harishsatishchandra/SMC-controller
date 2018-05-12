function [u] = AXController_pacejka(AX_REF,x,delta,p)
%AXController_pacejka Summary of this function goes here
%
% Syntax: AXController_pacejka
%
% Inputs:
%
% Outputs:
%
% See also: ---

% Author: Davide Calzolari
% Date: 2016/10/14 14:19:23
% Revision: 0.1
% Technische Universitaet Muenchen 2016

mu0=p.mu0;
R=p.R;
m=p.m;
vx=x(4);
vy=x(5);
omega=x(6);
l_F=p.l_F;
l_R=p.l_R;
C_R=p.C_R;
B_R=p.B_R;
L=p.L;
h=p.h;
J=p.J;
g=9.81;
Fb=( AX_REF - vy * omega ) * m;
Fxf=Fb;
Fxr=0;

% normal forces
fzf_max = (m*g*l_R)/(-mu0*h+L);  %the maximum normal force if all normal force is utilized for breaking
Fzf = max(0,min(fzf_max,l_R/L * m*g - h/L * Fb));
Fzr = m*g - Fzf;

% maximal absolute force
Fmaxf = mu0 * Fzf;
Fmaxr = mu0 * Fzr;

% velocity at rear axle
vr = [vx; vy - l_R * omega];
% velocity at front axle
vf = [ vx; vy + l_F * omega ];
vf_abs = norm(vf);

%rear lateral tire force
fr = @(omega_R)-mu0*Fmaxr*sin(C_R*atan(B_R*norm((vr-[R*omega_R;0])./norm(vr))/mu0))...
                        .*(vr-[R*omega_R;0])./norm(vr)/norm((vr-[R*omega_R;0])./norm(vr));
% if Fxr>0
%     omega_R0 = vx/R+0.001;
% else
%     omega_R0 = vx/R-0.001;
% end
%if Fxr==0
    omega_R = vx/R;
% else
%     omega_R = fminsearch(@(omega)([1,0]*fr(omega)-Fxr).^2,omega_R0);
% end
if norm((vr-[R*omega_R;0])./norm(vr))==0  
    Fyr = 0;
else
    Fyr = [0,1] * fr(omega_R);
end

%front lateral tire force
Fyf = 0;            %(m*(0+vx*omega) - 0*Fyr);

% absolute front tire force
Ff_abs = sqrt(Fxf*Fxf+Fyf*Fyf);

% maximal tire force exceeded?
if Ff_abs>Fmaxf
    %disp(['mu=' num2str(Ff_abs/Fmaxf)]);
    Fxf = Fxf * Fmaxf / Ff_abs;
    Fyf = Fyf * Fmaxf / Ff_abs;
    Ff_abs = Fmaxf;
end

ud = [Fxf;Fyf];

%u=ForceController_pacejka(ud,x,Ff_abs,Fmaxf,p);
B_F=p.B_F;
C_F=p.C_F;

%non commanded
omega_R = vx/R;

%% calculate required speed
sf_abs = mu0/B_F * tan( 1/C_F * asin( Ff_abs/Fmaxf ) );

% slip vector
if Ff_abs==0
    sf = zeros(2,1);
else
    sf = [ -Fxf/Ff_abs * sf_abs; -Fyf/Ff_abs * sf_abs ];
end
% velocity at front axle
vf = [ vx; vy + l_F * omega ];
vf_abs = norm(vf);
%Latsch velocity
Rot_delta=[cos(delta),-sin(delta);sin(delta),cos(delta)];
vLf = (sf * vf_abs - vf);
%RvLF = Rot_delta*vLf;
% required front wheel speed for slip along x
omega_F = -vLf(1)/R/Rot_delta(1,1);
%omega_F = norm(vLf)/R; 

%disp(['AX_REF: ' num2str(AX_REF)]);

u=[delta,omega_F,omega_R];
end



