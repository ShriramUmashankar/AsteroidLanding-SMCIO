function[a] = GetAccel(r_fd, v_fd, t_f, r, v, t, g, T_max , m ,a)
    t_go = t_f - t;
    ZEM = r_fd - (r + v*t_go + 0.5*g*t_go*t_go);
    ZEV = v_fd - (v + g*t_go);

    var1 = (6/t_go^2)*ZEM;
    var2 = (2/t_go)*ZEV;

    objective = @(a) norm(a)^2;

    % Constraints
    A = [];
    b = [];
    Aeq = eye(3);
    beq = var1 - var2 - g;   % Guidance law constraint
    
    % Lower and upper bounds (no specific bounds, so set as empty)
    lb = [];
    ub = [];
    
    % Thrust constraint as a nonlinear constraint
    nonlcon = @(a) thrust_constraint(a, T_max, m);
    options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
                           'EnableFeasibilityMode',true,...
                           'Display','off', ...
                           'TolFun', 1e-10, ...
                           'StepTolerance', 1e-12, ...
                           'ConstraintTolerance', 1e-6, ...
                           'MaxFunctionEvaluations', 100000, ...
                           'MaxIterations' , 100000 ...
                           );
    
    % Initial guess for acceleration [0, 0, 0]
    a0 = a;
    
    % Solve using fmincon
    a = fmincon(objective, a0, A, b, Aeq, beq, lb, ub, nonlcon, options);
        
    
end

function [c, ceq] = thrust_constraint(a, T_max, m)
    % Thrust magnitude constraint: ||a|| <= Tmax/m
    c = norm(a) - T_max / m;  % Inequality constraint (c <= 0)
    ceq = [];                 % No equality constraint
end

