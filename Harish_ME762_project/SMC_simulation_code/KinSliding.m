function [controller] = KinSliding()


controller.name='KINSM';
controller.control_point='REAR';
controller.init=@(model,options)init(controller,model,options);
end

function controller = init(controller,model,options)

n=length(model(0,zeros(10,1),zeros(5,1),options.p));

switch func2str(model)
    case options.types{1}
        mapInput=@AXController_pacejka;
    case options.types{2}
        mapInput=@map0;
end
controller.compute_input=@(varargin)compute_input(mapInput,n,varargin);
end

function [u] = compute_input(mapInput,n,varargin)

%reference trajectory
[state,tau,t,p] = parseInput(varargin{:});

% trajectory values
pd = [tau.X(t);tau.Y(t)];
vd = [tau.dX(t);tau.dY(t)]; 
thetad = tau.theta(t); 
omegad = tau.dtheta(t); 
domegad= tau.ddtheta(t);
lad = tau.a(t);


[e,de]=getLocalError(state,pd,vd,thetad,omegad,-p.l_R,2); 

%velocity of rear tire
vr=state(4); % == vx

%gains
k0=0.05;
k1=.25;
k2=.5;

%sliding surfaces   
s1=de(1)+k1*e(1);                      
s2=de(2)+k2*e(2)+k0*sgn(e(2))*e(3);    


p1=1;
p2=3;

q1=1;
q2=3;

%control law
dv_c=1/cos(e(3))*(-q1*s1-p1*sgn(s1)-k1*de(1)-domegad*e(2)-omegad*de(2)+... 
    vr*de(3)*sin(e(3))+lad);
delta=atan(p.L/vr*omegad+p.L/(vr*(vr*cos(e(3))+...                      
    k0*sgn(e(2))))*(-q2*s2-p2*sgn(s2)-k2*de(2)-... 
    dv_c*sin(e(3))+domegad*e(1)+omegad*de(1)));



delta=saturate( delta,-p.MAX_delta,p.MAX_delta);


u=mapInput(dv_c,state,delta,p);
u(end+1)=0;
end
