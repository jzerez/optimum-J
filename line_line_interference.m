function res = line_line_interference(x1, x2, xr, y1, y2, yr)
    res = true;
    min_dist = xr + yr;
    v = unit(x1 - x2);
    w = unit(y1 - y2);
    p = x1;
    q = y1;
    
    eqn = [-1, dot(w, v), dot((p-q),v);
           -dot(v, w), 1, dot((p-q), w)];
       
    sols = rref(eqn);
    n = sols(1, 3);
    m = sols(2, 3);
    
    
    p1 = p + n*v;
    q1 = q + m*w;
    
    pq = p1 - q1;
    dist = norm(pq);

    if dist > min_dist
        res = false;
        return
    else
        dists = [calc_point_line_dist(x1, x2, q1), ...
                 calc_point_line_dist(y1, y2, p1)];
        dist = min(dists);
        if dist > min_dist
            res = false;
            return
        end
    end
    
end