function res = line_line_interference(x1, x2, xr, y1, y2, yr)
    % This function calculates the shortest distance between 2 lines in 3d
    % space
    
    % x1, x2 are the endpoints for the first line
    % y1, y2 are the endpoints for the second line
    % xr, yr are the radii associated with each line
    
    res = true;
    min_dist = xr + yr;
    u = unit(x2 - x1);
    v = unit(y2 - y1);
    p = x1;
    q = y1;
    
    % Solve linear system of eqns based on the fact that the shortest
    % vector connecting two lines must be orthogonal to both lines
    eqn = [dot(u, u), -dot(u, v), -dot(u, p) + dot(q, u);...
           dot(u, v), -dot(v, v), -dot(v, p) + dot(q, v)];
       
    sols = rref(eqn);
    a = sols(1, 3);
    b = sols(2, 3);
    
    
    p1 = p + a*u;
    q1 = q + b*v;
    
    pq = p1 - q1;
    dist = norm(pq);

    if dist > min_dist
        res = false;
        return
    else
        % If the points of interest (p1, q1) along each line is outside the line
        % segment
        if a < 0 || b < 0 || a > norm(x2-x1) || b > norm(y2 - y1)
            res = false;
            return
        end
    end
    
end