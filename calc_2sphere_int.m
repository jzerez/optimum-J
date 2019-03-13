function [center, radius, plane] = calc_2sphere_int(center1, radius1, center2, radius2)
    c1 = center1;
    r1 = radius1;
    c2 = center2;
    r2 = radius2;
    
    delta = c2 - c1;
    d = norm(delta);
    axis = unit(delta);
    x = (r2^2 - r1^2 + d^2) / 2 / d;
    radius = sqrt(r1^2 - (d-x)^2);
    center = radius * axis + c1;
    plane = Plane(center, axis);
end