function [h] = generateNoise(vr,T,step)
%GENERATENOISE Summary of this function goes here
%
% Syntax: GENERATENOISE
%
% Inputs:
%
% Outputs:
%
% See also: ---

% Author: Davide Calzolari
% Date: 2016/10/17
% Revision: 0.1
% Technische Universität München 2016

TS=0:step:T;
dim=[length(TS),length(vr)];
mu = zeros(dim);
sigma = repmat(vr,length(TS),1);
v=normrnd(mu,sigma,dim);
h=@(t)myinterp(TS,v,t);

    function x = myinterp(t,x,ti)
        x = interp1(t,x,ti)';
    end

end
