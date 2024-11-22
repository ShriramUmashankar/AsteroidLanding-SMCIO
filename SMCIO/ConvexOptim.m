function [x] = ConvexOptim(V_l, V_u, a0, a1, a2, a3, v, g, h, m, T_max, T_min, a)
    % Objective function: Minimize sum of squares of x (or equivalently, z)
    fun = @(x) x(4);  % Minimize z directly
    
    % Define lower and upper bounds for the optimization variables x(1:3) and z
    lb_x = (V_l - v - g * h) / h;  % Bounds for x(1:3)
    ub_x = (V_u - v - g * h) / h;  % Bounds for x(1:3)
    lb_z = (T_min / m)^2;          % Lower bound for z
    ub_z = (T_max / m)^2;          % Upper bound for z
    
    % Concatenate bounds for x(1:3) and z
    lb = cat(2,lb_x,lb_z);
    ub = cat(2,ub_x,ub_z);

    % Linear equality constraint for x(1), x(2), x(3): a1*x(1) + a2*x(2) + a3*x(3) = a0
    Aeq = [a1 a2 a3 0];  % Include a zero coefficient for z
    beq = a0;
    
    % No inequality constraints (linear), so A and b are empty
    A = [];
    b = [];
    
    % Initial guess for x with the initial guess for z (set as norm of initial x guess)
    x0 = cat(2,a,norm(a)^2);
    
    % Define nonlinear constraint (using anonymous function to pass parameters)
    nonlcon = @(x) ccon_lifted(x);
    
    % Options for fmincon
    options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
                           'EnableFeasibilityMode', true, ...
                           'Display', 'off', ...
                           'TolFun', 1e-10, ...
                           'StepTolerance', 1e-12, ...
                           'ConstraintTolerance', 1e-6, ...
                           'MaxFunctionEvaluations', 100000, ...
                           'MaxIterations', 100000);
    
    % Call fmincon to solve the optimization problem
    [x, fval] = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);
end

% Nonlinear constraint function with lifted variable (for fmincon)
function [c, ceq] = ccon_lifted(x)
    % Constraint z = x(1)^2 + x(2)^2 + x(3)^2
    z = x(4);  % x(4) represents the lifted variable z
    c = [];
    
    % Nonlinear equality constraint ensuring z equals the squared thrust
    ceq = z - (x(1)^2 + x(2)^2 + x(3)^2);
end



