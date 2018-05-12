function [ p ] = CARparameters( )
%CARPARAMETERS 
p.m = 1750;
p.J = 2500;
p.l_F = 1.43;
p.l_R = 1.27;
p.L = p.l_F + p.l_R;
p.h = 0.5;
p.mu0 = 1;
p.B_F = 10.4;
p.C_F = 1.3;
p.Kfz_F = 0.1;
p.B_R = 21.4;
p.C_R = 1.1;
p.Kfz_R = 0.1;
p.R = 0.32;
p.Jwheel = 1.2;

%linear approximation values for control
p.cf=1.0299e+05;
p.cr=1.8917e+05;

%maximum values
p.MAX_delta=pi/4;

%controller A gains
p.kA0 = 5;
p.kA1 = 3.3541;

end

