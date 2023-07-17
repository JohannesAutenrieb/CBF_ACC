function [u, slack] = adaptiveCruiseController_ZCBF(x, Fr, V, h, params,sim)
%ADAPTIVECRUISECONTROLLER CLF-CBF QP-based adaptive cruise conroller
%implementation

% lie derivitavies of control lyapunov function
LfV     = -(2/params.m)*(x(2)-params.vd)*Fr + params.epsilon*V;
LgV     = (2/params.m)*(x(2)-params.vd);

% lie derivitavies of control barrier function
Lfh     = (params.Th/params.m)*Fr + (x(2)-params.v0) + params.gamma*h;
Lgh     = -(params.Th/params.m);

% lie derivitavies of force-based control barrier function
Lf_F_h     = (1/params.m)*(params.Th + (x(2)-params.v0)/(params.cd*params.g))*Fr + (params.v0-x(2)) + params.gamma*h;
Lg_F_h     = -(1/params.m)*(params.Th + (x(2)-params.v0)/(params.cd*params.g));



% Weighting matrix for minimizing the euclidean norm of the control input
% vector and penalizing the slack variable
H       = [params.weight.input    0;
            0               params.weight.slack];

% Compositve A and b matrix for all inequality constraints
if(sim.considerMinimumBreakingDistance)
    
    A       = [ LgV, -1;
            -Lgh,  0;
            -Lg_F_h,  0;
            1     0;
            -1    0];

    b       = [ -LfV;
                Lfh;
                Lf_F_h;
                params.ca*params.m*params.g;
                params.cd*params.m*params.g]; 
else
    A       = [ LgV, -1;
            -Lgh,  0;
            1     0;
            -1    0];

    b       = [ -LfV;
                Lfh;
                params.ca*params.m*params.g;
                params.cd*params.m*params.g];
end

   

 % set optimization options
options =  optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','off');

% Run QP-based optimization
[u_slack, ~, exitflag, ~] = quadprog(H, [], A, b, [], [], [], [], [], options);

% Return variables
u = u_slack(1);
slack = u_slack(2);
    
end