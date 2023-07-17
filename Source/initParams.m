function params = initParams
%INITSIM Initilizes all relevant simulation settings and simulation cases

%% Parameters are from 
% Aaron Ames et al. Control Barrier Function based Quadratic Programs 
% with Application to Adaptive Cruise Control, CDC 2014, Table 1.

params.g        = 9.81;
params.m        = 1650;
params.f0       = 0.1;
params.f1       = 5;
params.f2       = 0.25;
params.vd       = 24;
params.v0       = 14;
params.epsilon  = 0;
params.gamma    = 0.1;
params.ca       = 0.3;
params.cd       = 0.3;
params.psc      = 1e-5;
params.Th       = 1.8;
params.u_max    = params.ca * params.m * params.g;
params.u_min    = -params.cd * params.m * params.g;

params.p_1      = 1;
params.p_2      = 1;

params.weight.input = 2/params.m^2;
params.weight.slack = 1000000*2*params.psc;

end

