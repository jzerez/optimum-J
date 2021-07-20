function [p1, p2] = calc_3sphere_int(center1, radius1, center2, radius2, center3, radius3)
    [circle_center, circle_radius, circle_plane] = calc_2sphere_int(center1, radius1, center2, radius2);
    v1, [p1, p2] = calc_sphere_circle_int(center3, radius3, circle_center, circle_radius, circle_plane);
end