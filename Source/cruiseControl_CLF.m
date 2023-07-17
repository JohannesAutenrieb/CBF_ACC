function V = cruiseControl_CLF(x, params)
%CRUISECONTROL_CLF Computes the value of the chosen control lyapunov
%function candidate used for the CLF-based cruise control

V = (x(2)-params.vd)^2;
    
end

