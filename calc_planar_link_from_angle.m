function point = calc_planar_link_from_angle(plane, center1, radius1, center2, angle)
    % This function finds the intersection point between two line segments
    % such that one forms a specific angle with respect to the line that
    % connects the two known segment endpoitns. 
    
    % This script is based on the law of sines. a/sin(A) = b/sin(B)
    
    % plane is the plane that everything is constrained to
    % center1 is the fixed endpoint of the first segment
    % radius1 is the fixed length of the first segment
    % center2 is the fixed endpoint of the second segment
    % angle is the angle in degrees between the line from center1 to
    %   center2 and the first segment
    
    assert(plane.is_in_plane(center1));
    assert(plane.is_in_plane(center2));
    
    center1_2d = plane.convert_to_planar_coor(center1);
    center2_2d = plane.convert_to_planar_coor(center2);
    
    % Center coordinate system around the first center
    offset = center1_2d;
    center1_2d = center1_2d - offset;
    center2_2d = center2_2d - offset;
    
    % Check for degenerate cases of angles close to  0, 180, 360, etc
    % Those angles should be restricted by the constraints set in Optimizer
%     assert(mod(angle, 180) > 5 && mod(angle, 180) < 175);
    
    theta0 = atan2d(center2_2d(2), center2_2d(1));
    
    point = [radius1*cosd(theta0 + angle); radius1*sind(theta0 + angle)];
    point = plane.convert_to_global_coor(point + offset);
end