function fitness = calc_fitness(static_char, dyn_char, desired_static_char, desired_dyn_char, static_char_weights, dyn_char_weights)
    scf = fieldnames(static_char);
    dcf = fieldnames(dyn_char);
    
    static_error = 0;
    dynamic_error = 0;
    for index = 1:numel(scf)
        i = scf{index};
        % Loose consistency check for struct properties
        assert(isfield(desired_static_char, i));
        assert(isfield(static_char_weights, i));
        
        % Assign fitness based on weight (simple)
        error = static_char_weights(i) * abs(desired_static_char(i) - static_char(i)) / desired_static_char(i);
        static_error = static_error + error;
    end
    % Calc total wheel travel
    s = size(dyn_char.wheel_centers);
    v1 = dyn_char.wheel_centers(1, round(s(2)/2));
    v2 = dyn_char.wheel_centers(end, round(s(2)/2));
    wheel_travel = abs(v2(2) - v1(2));
    travel_error = abs(wheel_travel - 2.25) * dyn_char_weights.wheel_travel;
    dynamic_error = dynamic_error + travel_error;
    
    % Calc avg. bump steer
    top_plane = Plane([0;0;0], [1;0;0], [0;0;1]);
    right_plane = Plane([0;0;0], [0;1;0], [0;0;1]);
    num_shock_pos = size(dyn_char.wheel_positions, 3);
    num_rack_pos = size(dyn_char.wheel_postiions, 2);
    toe_angles = zeros([num_shock_pos, num_rack_pos]);
    camber_angles = zeros([num_shock_pos, num_rack_pos]);
    lat_pos = zeros([num_shock_pos, num_rack_pos]);
    
    for shock_ind = 1:length(num_shock_pos)
        for steer_ind = 1:length(num_rack_pos)
            pos = dyn_char.wheel_centers(:, steer_ind, shock_ind);
            lat_pos(shock_ind, rack_ind) = pos(1);
            ori = dyn_char.wheel_orientations(:, steer_ind, shock_ind);
            toe_vec = unit(top_plane.project_into_plane(ori));
            camber_vec = unit(right_plane.project_into_plane(ori));
            toe_cr = cross(toe_vec, [0;0;1]);
            camber_cr = cross(camber_vec, [0;1;0]);
            toe_angles(shock_ind, rack_ind) = acosd(dot(toe_vec, [0; 0; 1])) * sign(toe_cr(2));
            camber_angles(shock_ind, rack_ind) = acosd(dot(camber_vec, [0;1;0])) * sign(camber_cr(1));
        end
    end
    avg_bump_steer = mean(mean(diff(toe_angles)));
    bump_steer_error = avg_bump_steer * dyn_char_weights.bump_steer;
    dynamic_error = dynamic_error + bump_steer_error;
    
    % Calc avg. scrub in/out with shock compression
    top = lat_pos(1, :);
    bot = lat_pos(end, :);
    total_avg_scrub = mean(abs(top-bot));
    scrub_error = total_avg_scrub * dyn_char_weights.scrub;
    dynamic_error = dynamic_error + scrub_error;
    
    fitness = dynamic_error + static_error;
    
    

end