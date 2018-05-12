function [x0] = getVehicleX0(model,tau,options)

n=int32(length(model(0,zeros(10,1),zeros(5,1),options.p)));

x0 = zeros(n,1);
x0(1)=tau.X(0);
x0(2)=tau.Y(0);
x0(3)=tau.theta(0);
x0(4)=tau.v(0);
x0(5)=0;
x0(6)=0;

switch n
    case 9 %5DOF model
        w0=tau.v(0)/options.p.R; %wheel speed
        x0(7)=w0;
        x0(8)=w0;
    case 7
        %x0(1)=tau.X(0)-(options.p.l_R);
end


end
