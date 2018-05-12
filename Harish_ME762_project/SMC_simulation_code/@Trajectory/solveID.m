function solveID(obj,p)

    f = @(t,x)obj.internalDynamics_vehicle_A(x,p,obj.dX(t),obj.ddX(t),obj.dY(t),obj.ddY(t));
    [TXIode,XIode] = ode113(f,0:0.01:obj.T,[0;0],odeset('AbsTol',1e-10,'RelTol',1e-10));
    for i=1:length(TXIode)
        dxi = f(TXIode(i),XIode(i,1:2));
        XIode(i,3) = dxi(2);
    end
    obj.psi0 = @(t)interp1(TXIode,XIode(:,1),t,'spline');
    obj.dpsi0 = @(t)interp1(TXIode,XIode(:,2),t,'spline');
    obj.ddpsi0 = @(t)interp1(TXIode,XIode(:,3),t,'spline');
end
