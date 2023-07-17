function Fr = aerodynamicDrag(x, params)
%AERODYNAMICDRAG Computes the currently attacking drag force on the vehicle
%as a function of the current velocity and experimental data

Fr = params.f0 + params.f1*x(2) + params.f2*x(2)*x(2);
    
end

