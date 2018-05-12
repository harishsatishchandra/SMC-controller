function [ e,de ] = getLocalError( state, pd,vd,thetad,wd, DL, varargin)
%GETLOCALERROR 
x=state(1);
y=state(2);
psi=state(3);
vx=state(4);
vy=state(5);
omega=state(6);

ref=1; %default: local vehicle frame
if nargin > 6
    ref=varargin{1}; %1:local vehicle frame, 2:local desired vehicle frame
end
if ref==1
    rot_z=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    drot_z=skew([0,0,omega]')*rot_z;
else if ref==2
        rot_z=[cos(thetad) -sin(thetad) 0; sin(thetad) cos(thetad) 0; 0 0 1];
        drot_z=skew([0,0,wd]')*rot_z;
    else
        error('Not a valid frame');
    end
end

e_cart=[[x+DL*cos(psi);y+DL*sin(psi)]-pd; psi-thetad];
e=rot_z.'*e_cart;
Rot=[cos(psi) -sin(psi); sin(psi) cos(psi)];
v_xy_cart=Rot*[vx;vy] + [-sin(psi);cos(psi)]*DL*omega;
de=rot_z.'*[v_xy_cart-vd; omega-wd] + drot_z.'*e_cart;

end

