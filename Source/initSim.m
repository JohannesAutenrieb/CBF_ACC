function sim= initSim
%INITSIM Initilizes all relevant simulation settings and simulation cases

% define time steps and total time of simulation
sim.dt = 0.02;
sim.sim_t = 100;

% consider minimum braking distance required to decelerate from v to v0
sim.considerMinimumBreakingDistance = 1;

% checks results folder existis and creates path later used to save plots
sim.resultsPath = generateResultsPath(sim);

end

