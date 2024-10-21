function [x] = ConvexOptim(V_l, V_u, a0, a1, a2, a3, v, g, h, m, T_max, a)
    % Objective function: Minimize sum of squares of x
    fun = @(x) x(1)^2 + x(2)^2 + x(3)^2;
    
    % Define lower and upper bounds for the optimization variables

    lb = (V_l - v - g*h)/h;
    ub = (V_u - v - g*h)/h;
    
    % Linear equality constraint: a1*x(1) + a2*x(2) + a3*x(3) = a0
    Aeq = [a1 a2 a3];
    beq = a0;
    
    % No inequality constraints (linear), so A and b are empty
    A = [];
    b = [];
    
    % Initial guess for x
    x0 = a; 
    
    % Define nonlinear constraint (using anonymous function to pass parameters)
    nonlcon = @(x) ccon(x, T_max, m);
    options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
                           'EnableFeasibilityMode',true,...
                           'Display','off', ...
                           'TolFun', 1e-10, ...
                           'StepTolerance', 1e-12, ...
                           'ConstraintTolerance', 1e-6, ...
                           'MaxFunctionEvaluations', 100000, ...
                           'MaxIterations' , 100000 ...
                           );
    
    % Call fmincon to solve the optimization problem
    x = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon,options);
end

% Nonlinear constraint function (for fmincon)
function [c, ceq] = ccon(x, T_max, m)
    % Inequality constraint: sum of squares of x should be <= (T_max/m)^2
    c = x(1)^2 + x(2)^2 + x(3)^2 - (T_max/m)^2;
    
    % No nonlinear equality constraints
    ceq = [];
end
