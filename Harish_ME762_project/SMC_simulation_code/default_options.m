function [ options ] = default_options()
%DEFAULT_OPTIONS 

options.DISPLAY_OUTPUT=1;
options.V=0; %noise
CARparameters;

options.p=CARparameters;
options.pc=CARparameters;
%time span
options.stepSize=0.02;
%model types
options.types={'vmodel_A','vmodel_K'};
end

