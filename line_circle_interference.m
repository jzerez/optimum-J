function res = line_circle_interference(center, radius, plane, x1, x2, xr)
    if plane.calc_norm_dist(x1) * plane.calc_norm_dist(x2) > 0
        res = false;
        return;
    end
    v = unit(x2 - x1);
    norm_vec = plane.calc_norm_dist(x1) * plane.normal;
    n = -dot(norm_vec, (x1 - plane.position)) / dot(norm_vec, v);
    int = x1 + n*v;
    assert(plane.is_in_plane(int))
    norm(int-center)
    if norm(int - center) > (radius - xr)
        res = true;
        return
    end
    res = false;
    
end