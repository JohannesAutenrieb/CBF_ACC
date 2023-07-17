% Simple cruise control example for a study on the application of CBF and
% CLF in the control of nonlinear systems
%
%*************************************************************************
%
% Filename:				AdaptiveCruiseControl_CLF_CBF.m
%
% Author:				Johannes Autenrieb, johannes.autenrieb@outlook.com
% Created:				26-Aug-2022
%
%*************************************************************************
%
% Description:
%		Simple cruise control example for a study on the application of CBF 
%       and CLF in the control of nonlinear systems based on the paper of 
%       A. D. Ames, J. W. Grizzle, and P. Tabuada, “Control barrier 
%       function based quadratic programs with application to adaptive 
%       cruise control,” CDC 2014
%
% Input parameter:
%		- none
%		- none
%
% Output parameter:
%		- none
%		- none
%
%% #######################    SCRIPT START   ############################

%% Pre-steps
% clear workspace
clear all; close all;

% add export_fig lib
addpath(genpath(['.' filesep '..' filesep 'Source']));
addpath(genpath(['.' filesep '..' filesep 'Include']));

%% init all relevant simulation settings and simulation cases
sim = initSim;

%% init all relevant parameter of the regarded case and model
params = initParams;

%% init state of vehicle
x0 = [0; 20; 100];

%% initialize and pre-allocate run time variables.
total_k = ceil(sim.sim_t / sim.dt);
x = x0;
t = 0;   

xs = zeros(total_k, 3);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
slacks = zeros(total_k-1, 1);
hs = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
xs(1, :) = x0';
ts(1) = t;
v = x(2);

%% run simulation as loop
for k = 1:total_k
    
    %% Computing relevant information for current time step
    
    % computing current CBF value based on regarded CBF type

    h       = safetyDistance_ZCBF(x, params,sim);  
    
    % computing current CLF value
    V       = cruiseControl_CLF(x, params); 
    
    % computing the aerodynamics drag
    Fr      = aerodynamicDrag(x, params);
    
    %% Computing needed control input to get desired cruise behaviour
    

    [u, slack]      = adaptiveCruiseController_ZCBF(x, Fr, V, h, params,sim);
    
    % Saving control relevant information of current time step
    us(k, :) = u';
    slacks(k, :) = slack;
    
    % save CBF information
    hs(k) = h;
    
    Vs(k) = V;
    
    % Run one time step propagation.
    [ts_temp, xs_temp] = ode45(@(t, s) cruiseDynamics(x,u, params), [t t+sim.dt], x);
    x = xs_temp(end, :)';
    
    % saving the state of current time step
    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    
    % Setting time for next time step
    t = t + sim.dt;
    
end

%% plot results

% plot results
plotResults_ZCBF(ts, xs, us, slacks, hs, Vs, params, sim)


% #######################     SCRIPT END    ############################