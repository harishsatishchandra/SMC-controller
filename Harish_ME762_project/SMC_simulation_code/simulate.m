function [benchmark] = simulate(tau,tauD,controller,model,x0,options)
%%	INITIALIZATION PROCEDURE
%   time step
h=options.stepSize;
%real parameters
p=options.p;
%parameter known to the controller
pc=options.pc;
%noise variables
xi_m=generateNoise(options.V,tau.T,options.stepSize);


tspan=0:h:tau.T;
isDiscrete=(isfield(controller,'type') && strcmp(controller.type,'discrete'));

tic
%profile on
if isDiscrete
    disp('discrete')
    odef=@(t,x,newTick,intState)fullsystemDISC(t,x,@(t)0,xi_m,tauD,model,controller,p,pc,newTick,intState);
    intState0=controller.intState0;
    [T,X,intState]=myode4_fixed_state(odef,tspan,x0,intState0);
    T=T';
else
    disp('continuous')
    odef=@(t,x)model(t,x,controller.compute_input(x+xi_m(t),tauD,t,pc),p);
    [T,X]=ode45(odef,tspan,x0);
    T=reshape(T,[length(T) 1]);
end
%profile off
toc


if isDiscrete
    U=zeros(size(T,1),length(controller.compute_input(X(1,:)',tauD,T(1),pc,0,intState{1})));
else
    U=zeros(size(T,1),length(controller.compute_input(X(1,:)',tauD,T(1),pc)));
end

ERROR=zeros(size(T,1),2);
MUXY=zeros(size(T,1),2);
for k=1:size(T,1)
    %input
    if isDiscrete
        [U(k,:)]=controller.compute_input(X(k,:)',tauD,T(k),pc,0,intState{k});
    else
        [U(k,:)]=controller.compute_input(X(k,:)',tauD,T(k),pc);
    end
    %error
    Xd = tau.X(T(k)); Yd = tau.Y(T(k)); theta=tau.theta(T(k));
    ct = cos(theta); st = sin(theta);
    Rti = [ct,st;-st,ct];
    ERROR(k,:)=Rti*(X(k,1:2)'-[Xd,Yd]');
    %tire saturation
    [~,y]=model(0,X(k,:)',U(k,:),p);
    sf = y(1:2);
    sr = y(3:4);
    MUXY(k,:)=[norm(sf), norm(sr)];
end
ET=(ERROR(:,1));
EN=(ERROR(:,2));
MUF=(MUXY(:,1));
MUR=(MUXY(:,2));
%calculate averages
IntET = trapz(T,abs(ET))/tau.T;
IntEN = trapz(T,abs(EN))/tau.T;
IntMUF = trapz(T,MUF)/tau.T;
IntMUR = trapz(T,MUR)/tau.T;


benchmark.controller=controller.name;
benchmark.model=model;
benchmark.scenario=tau.id;
benchmark.max_error=[max(abs(ET)) max(abs(EN))];
benchmark.avg_error=[IntET IntEN];%mean(abs(ERROR));
benchmark.end_error=[ET(end) EN(end)];
benchmark.avg_musat=[IntMUF IntMUR];%mean(MUXY);
benchmark.total_time=T(end);
benchmark.data.error=[ET EN];
benchmark.data.U=U;
if (isDiscrete) benchmark.data.intState=intState; end
benchmark.data.control_point=controller.control_point;
benchmark.data.X=X;
benchmark.data.T=T;
benchmark.data.tau=tau;
benchmark.data.tauD=tauD;
benchmark.data.options=options;
end


function [dx,intState] = fullsystemDISC(t,x,xi_dx,xi_m,tauD,model,controller,p,pc,newTick,intState)
% t - time
% x - state
% xi_dx - process disturbance
% xi_m - measurement error
% xi_p - parameter mismatch
[u,intState] = controller.compute_input(x+xi_m(t),tauD,t,pc,newTick,intState);
[dx,y] = model(t,x,u,p);
dx = dx + xi_dx(t);
end
