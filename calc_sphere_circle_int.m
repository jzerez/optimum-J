function [v1, v2] = calc_sphere_circle_int(sphere_center, sphere_radius, circle_center, circle_radius, circle_plane)
    % Takes 3D sphere and 3D circle and finds the intersection between them
    sphere_center3d = circle_plane.project_into_plane(sphere_center);
    sphere_center2d = circle_plane.convert_to_planar_coor(sphere_center3d);
    sphere_eff_radius = sqrt(sphere_radius^2 - circle_plane.calc_norm_dist(sphere_center)^2);

    circle_center2d = circle_plane.convert_to_planar_coor(circle_center);

    disp(circle_radius)
    [x,y] = circcirc(sphere_center2d(1), sphere_center2d(2), sphere_eff_radius, ...
                     circle_center2d(1), circle_center2d(2), circle_radius);

    v1 = [x(1); y(1)];
    v1 = circle_plane.convert_to_global_coor(v1);
    v2 = [x(2); y(2)];
    v2 = circle_plane.convert_to_global_coor(v2);
end