classdef Rocker
    properties
        pivot_point;
        shock_point;
        shock_lever;
        shock_index;
        control_arm_point;
        control_arm_lever;
        control_arm_index;
        arb_point;
        arb_lever;
        arb_index;
        plane;
        ordered_points;
        
        rotation_direction;
    end
    
    methods
        function self = Rocker(pivot_point, shock_point, control_arm_point, arb_point)
            self.plane = Plane(pivot_point, shock_point, control_arm_point);
            assert(self.plane.is_in_plane(arb_point));
            self.shock_lever= norm(shock_point - pivot_point);
            self.control_arm_lever = norm(control_arm_point - pivot_point);
            self.arb_lever = norm(arb_point - pivot_point);
            self.shock_point = shock_point;
            self.control_arm_point = control_arm_point;
            self.arb_point = arb_point;
            self.pivot_point = pivot_point;
            
            % sort points into a closed polygon
            all_points = [pivot_point, shock_point, control_arm_point, arb_point];
            moveable_points = all_points(:, 2:end);
            avg_point = mean(all_points, 2);
            reference = mean(pivot_point - avg_point)*ones([1, 3]);
            direction = vecnorm(moveable_points - avg_point);
            unsigned_angles = acosd(dot(reference, direction));
            signed_angle_offset = (cross(reference, direction) < 0)*180;
            signed_angles = signed_angle_offset + unsigned_angles;
            [~, I] = sort(signed_angles);
            self.ordered_points = [self.pivot_point, moveable_points(:, I)];
            % NEED TO FIX!!!! CONTROL ARM AND ARB INDEXES ARE MIXED UP
            self.shock_index = find(I==1)+1;
            self.control_arm_index = find(I==2)+1;
            self.arb_index = find(I==3)+1;
            
            self.rotation_direction = 1;
        end
        
        function self = rotate(self, thetad, new_shock_point)
            axis = self.plane.normal;
            centered_points = self.ordered_points - self.pivot_point;
            angled = thetad / 2 * self.rotation_direction;
            real = cosd(angled);
            imag = sind(angled) * axis;
            q = quaternion(real, imag(1), imag(2), imag(3));
            new_points = rotatepoint(q, centered_points');
            self.ordered_points = new_points' + self.pivot_point;
            
            self.shock_point = self.ordered_points(:, self.shock_index);
            self.control_arm_point = self.ordered_points(:, self.control_arm_index);
            self.arb_point = self.ordered_points(:, self.arb_index);
            if ~isnan(new_shock_point)
                if norm(self.shock_point - new_shock_point) > 0.0001
                    disp('SWITCHING ROTATION')
                    self.rotation_direction = -self.rotation_direction;
                    self = self.rotate(thetad * 2, NaN);
                end
            end
            
        end
    end
end