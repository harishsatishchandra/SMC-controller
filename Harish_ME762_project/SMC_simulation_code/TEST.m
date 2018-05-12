% addpath( genpath( cd ) );
function [ benchmark ] = TEST()
%TEST

%specify scenario: 01_single_lane_change or 02_double_lane_change
scenario = '01_single_lane_change';
controller = KinSliding;
model = @vmodel_A;
options = default_options;
options.p.mu0=1;
options.pc.mu0=1;

tau = Trajectory();
tau.load(['Scenarios/' scenario]);

if strcmp(controller.control_point,'CO')
    tau.solveID(options.pc);
    tauD = tau.transform(options.p.J/options.p.l_R/options.p.m).make_static(100);
else if strcmp(controller.control_point,'REAR')
        tau.solveID(options.pc);
        tauD = tau.transform(-options.p.l_R).make_static(100);
    else
        tauD=tau;
    end
end

%scale cf,cr
if (options.pc.mu0<1)
    options.p.cf=options.p.cf*options.p.mu0;
    options.p.cr=options.p.cr*options.p.mu0;
    options.pc.cf=options.pc.cf*options.pc.mu0;
    options.pc.cr=options.pc.cr*options.pc.mu0;
end

% options.p.m=1.3*options.p.m;
% options.p.J=1.3*options.p.J;
% options.p.l_R=1.3*options.p.l_R;
% options.p.L=options.p.l_R+options.p.l_F;

%tau=load_path(scenario,options.duration);
%initial state
options.n=int32(length(model(0,zeros(10,1),zeros(5,1),options.p)));
x0=getVehicleX0(model,tau,options)+0*[0 -0.2 degtorad(-3) 0 0 0 0]';

%initialize controller;
controller=controller.init(model,options);

%run
%profile on
benchmark=simulate(tau,tauD,controller,model,x0,options);
%profile off
%profile viewer

%visualize
if options.DISPLAY_OUTPUT
    disp(benchmark)
    % draw real time plot
    visualize(benchmark)
end
end

