function h = safetyDistance_ZCBF(x, params, sim)
%CONTROLBARRIERFUNCTION Computes the value of the chosen control barrier
%function candidate used for the CBF-based safe set invariance

if(sim.considerMinimumBreakingDistance)
    h = x(3)-params.Th*x(2)- 0.5  * ((params.v0-x(2))^2) / (params.cd * params.g);
else
    h = x(3)-params.Th*x(2);
end
    
end