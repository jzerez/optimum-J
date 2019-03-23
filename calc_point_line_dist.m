function dist = calc_point_line_dist(p1, p2, q)
    v = unit(p2 - p1);
    d1 = norm(p2 - p1);
    midpoint = p1 + v*(d1/2);
    
    w = q - midpoint;
    parallel_vec = dot(w, v)*v;
    norm_vec = w - parallel_vec;
    if norm(norm_vec) == 0
        dist = 0;
        return
    elseif norm(parallel_vec) > d1/2
        dist = min([norm(p1 - q), norm(p2 - q)]);
    else
        dist = norm(norm_vec);
    end
    
end