function point = find_closer_point(point_of_interest, p1, p2)
    dist1 = norm(point_of_interest - p1);
    dist2 = norm(point_of_interest - p2);

    if dist1 < dist2
        point = p1;
    else
        point = p2;
    end
end