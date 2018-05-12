function [ measure ] = measr(t,y,Q,seed)
%MEASR
%   Q: diagonal error variance matrix
%   seed: phase vector
%
%   add measurments noise
%   needs to be function of time so that ode45 can solve it
% w=sqrt(2.*Q)*sin(repmat(t'*1e2,length(y),1)+repmat(seed,1,length(t)));
% measure=repmat(y,1,length(t))+w;
w=sqrt(2.*Q)*sin(t*1e2+seed);
measure=y+w;

