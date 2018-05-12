classdef Trajectory<handle
   properties   
    id          % identifier
    T           % End-Time
    s           % arc-length
    v           % path-tangential velocity
    a           % derivative of path-tangential velocity
    j           % second derivative of path-tangential velocity
    theta       % velocity orientation
    kappa       % path curvature
    dtheta      % derivative of velocity orientation
    ddtheta     % second derivative of "
    dddtheta    % third derivative of "
    X           % earth-fixed x- and y-coordinates, and derivatives...
    Y           
    dX
    dY
    ddX
    ddY
    psi0        % internal dynamics solution: orientation of vehicle
    dpsi0       
    ddpsi0
    lambda      % transformation distance
   end
   methods 
       % constructor
       function obj = Trajectory()
           obj.lambda = 0;
       end
   end
   methods
      % load variables gx,gy,gs with poly-constraints from .mat-file and
      % create a trajectory representation
      load(obj,tau_file);
      % creates a static, linear interpolated representation
      obj_static = make_static(obj,res);
      % transforms the trajectory for the trajectory of a reference position
      objD = transform(obj,lambda);
      % solve internal dynamics
      solveID(obj,p);
   end
   methods (Static)
      [S,W] = arcLength(X,Y);
      dxi = internalDynamics_vehicle_A(xi,p,dX,ddX,dY,ddY);
   end
end