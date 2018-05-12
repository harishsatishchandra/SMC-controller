function [x,tau,t,p,newTick,intState] = parseInput(input)
%EXTRACTINPUT Summary of this function goes here
%
% Syntax: EXTRACTINPUT
%
% Inputs:
%
% Outputs:
%
% See also: ---

% Author: Davide Calzolari
% Date: 2016/09/16 13:07:02
% Revision: 0.1
% Technische Universitaet Muenchen 2016

newTick=1;

switch length(input)
    case 3
        % dicrete version for CORA
        x=input{1};
        tauREF=input{2};
        p=input{3};
        %reference trajectory
        pd=tauREF(1:2);
        vd=tauREF(3:4);
        ad=tauREF(5:6);
        jd=tauREF(7:8);

        tau.X=pd(1);
        tau.Y=pd(2);
        tau.dX=vd(1);
        tau.dY=vd(2);
        tau.ddX=ad(1);
        tau.ddY=ad(2);
        %path info
        tau.dtheta=(ad(2)*vd(1)-ad(1)*vd(2))/(vd(1)^2+vd(2)^2);
        tau.theta=atan2(vd(2),vd(1));
        % %desired longitudinal speed
        tau.v=sqrt(vd(1)^2+vd(2)^2);
        tau.a=(vd(1)*ad(1)+vd(2)*ad(2))/sqrt(vd(1)^2+vd(2)^2);
        t=1; %first element
        %if length(ref)>8
        %measurement noise
        v=tauREF(9:end);
        x=x+v;
        %end
    case 4
        x=input{1};
        tau=input{2};
        t=input{3};
        p=input{4};
        %         %reference trajectory
        %         [pd,vd,ad,jd]=get_trajectory(tau,t);
        %
        %         th=atan2(vd(2),vd(1));
        %         dth=(ad(2)*vd(1)-ad(1)*vd(2))/(vd(1)^2+vd(2)^2);
        %         ddth=d2dt2_atan(vd(2),vd(1),ad(2),ad(1),jd(2),jd(1));
        %     otherwise
        %         error('wrong numer of arguments');
        case 6 %for discrete systems
        x=input{1};
        tau=input{2};
        t=input{3};
        p=input{4};
        newTick=input{5};
        intState=input{6};
end

end
