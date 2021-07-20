function fitness = calc_fitness(static_char, dyn_char, desired_static_char, static_char_weights, desired_dyn_char, dyn_char_weights)
    % Static Characteristic Error Calculations
    scf = fieldnames(static_char);
    static_error = 0;
    
    for index = 1:numel(scf)
        i = scf{index};

        % Loose consistency check for struct properties
        assert(isfield(desired_static_char, i));
        assert(isfield(static_char_weights, i));

        % Assign fitness based on weight (simple)
        char = static_char.(i);
        normalizer = desired_static_char.(i);
        if normalizer == 0
            normalizer = 1;
        end
        error = static_char_weights.(i) * abs(desired_static_char.(i) - char) / normalizer;
        static_error = static_error + error;
    end
    
    % Dynamic Characteristics Error Calculations
    dcf = fieldnames(dyn_char);
    dynamic_error = 0;
    
    % Wheel Travel Error %
    s = size(dyn_char.toes, 2);
    % Get the top and bottom contact patch locations at zero toe
    v1 = dyn_char.contact_patches(1, round(s/2), :);
    v2 = dyn_char.contact_patches(end, round(s/2), :);
    % Find vertical dist
    wheel_travel = abs(v2(2) - v1(2));
    if wheel_travel < desired_dyn_char.min_wheel_travel
        travel_error = dyn_char_weights.min_wheel_travel;
    else
        travel_error = abs(wheel_travel - desired_dyn_char.wheel_travel) * dyn_char_weights.wheel_travel;
    end
    dynamic_error = dynamic_error + travel_error;

    
    % Bump Steer Error %
    avg_bump_steer = mean(mean(diff(dyn_char.toes)));
    bump_steer_error = abs(avg_bump_steer - desired_dyn_char.bump_steer) * dyn_char_weights.bump_steer;
    dynamic_error = dynamic_error + bump_steer_error;

    % Lateral Scrub Error %
    lateral_pos = dyn_char.contact_patches(:, :, 1);
    avg_scrub = mean(mean(diff(lateral_pos + min(lateral_pos)*ones(s, 1))));
    scrub_error = abs(avg_scrub - desired_dyn_char.scrub) * dyn_char_weights.scrub;
    dynamic_error = scrub_error + dynamic_error;
    
    % max steering error %
    max_steer = mean(maxk(reshape(dyn_char.toes, [1, numel(dyn_char.toes)]), 3));
    max_steer_error = (desired_dyn_char.max_steer_angle > max_steer) * dyn_char_weights.max_steer_angle;
    dynamic_error = max_steer_error + dynamic_error;
    
    fitness = dynamic_error + static_error;
end