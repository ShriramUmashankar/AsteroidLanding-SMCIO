function [value, isterminal, direction] = event_terminal(t, y)
    value = y(1)-1;        % When t equals t_terminal
    isterminal = 1;          % Stop integration
    direction = 0;           % Any direction
end