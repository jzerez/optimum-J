function point = calc_planar_link_from_angle(plane, center1, radius1, center2, angle)
    % This function finds the intersection point between two line segments
    % such that they form a specific angle. Both line segments must have points
    % defined in space, and one must have a set length
    
    % This script is based on the law of sines. a/sin(A) = b/sin(B)
    
    % plane is the plane that everything is constrained to
    % center1 is the fixed endpoint of the first segment
    % radius1 is the fixed length of the first segment
    % center2 is the fixed endpoint of the second segment
    % angle is the angle in degrees between the two segments
    
    assert(plane.is_in_plane(center1));
    assert(plane.is_in_plane(center2));
    
    center1_2d = plane.convert_to_planar_coor(center1);
    center2_2d = plane.convert_to_planar_coor(center2);
    
    % Check for the degenerate case for sin(angle) = 0
    if mod(angle, 180) == 0
        v = unit(center2_2d - center1_2d);
        point = plane.convert_to_global_coor(center1_2d + (v * radius1));
        return;
    end
    
    % Use law of sines to find the length of the second segment
    straight_dist = norm(center1_2d - center2_2d);
    law_of_sines_constant = straight_dist / sind(angle);
    angle2 = asind(radius1 / law_of_sines_constant);
    angle3 = 180 - angle2 - angle;
    radius2 = law_of_sines_constant * sind(angle3);
    
    % Find the intersection between two circles of known radii and centers
    [xs, ys] = circcirc(center1_2d(1), center1_2d(2), radius1, center2_2d(1), center2_2d(2), radius2);
    % convert back into 3d space
    points_2d = [xs;ys];
    points = zeros([3, 2]);
    points(:, 1) = plane.convert_to_global_coor(points_2d(:, 1));
    points(:, 2) = plane.convert_to_global_coor(points_2d(:, 2));
    
    % take cross product to determine angle sign
    cross_direction = cross((center2 - points(:, 1)), (center1 - points(:, 1)));
    if sign(angle) == sign(dot(unit(cross_direction), plane.normal))
        point = points(:, 1);
    else
        point = points(:, 2);
    end
    
end