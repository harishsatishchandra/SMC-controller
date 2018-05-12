function [u] = ForceController_pacejka(ud,x,Ff_abs,Fmaxf,p)
%FORCECONTROLLERPACEJKA
% 
% Syntax: FORCECONTROLLERPACEJKA 
% 
% Inputs: 
% 
% Outputs: 
% 
% See also: ---

% Authors: Daniel Heß (Original Author), Davide Calzolari 
% Date: 2016/10/15
% Revision: 0.1 
% Technische Universitaet Muenchen 2016

Fxf=ud(1);
Fyf=ud(2);

mu0=p.mu0;
R=p.R;
vx=x(4);
vy=x(5);
omega=x(6);
B_F=p.B_F;
C_F=p.C_F;
l_F=p.l_F;

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
vLf = sf * vf_abs - vf;
% required front wheel speed
omega_F = norm(vLf)/R;
% steering angle
delta = atan2( -vLf(2), -vLf(1) );

u=[delta,omega_F,omega_R];

end
