classdef Rocker < handle
    properties
        pivot_node;
        shock_node;
        control_arm_node;
        
        shock_lever;
        shock_index = 2;
        control_arm_lever;
        control_arm_index = 3;
        plane;
        ordered_points;
        
        rotation_direction;
    end
    
    methods (Access = public)
        function self = Rocker(pivot_node, shock_node, control_arm_node)
            self.rotation_direction = 1;
            self.shock_node = shock_node;
            self.control_arm_node = control_arm_node;
            self.pivot_node = pivot_node;
            
            self.update();
        end
        
        function rotate(self, thetad, new_shock_point)
            axis = self.plane.normal;
            centered_points = self.ordered_points - self.pivot_node.location;
            angled = thetad / 2 * self.rotation_direction;
            real = cosd(angled);
            imag = sind(angled) * axis;
            q = quaternion(real, imag(1), imag(2), imag(3));
            new_points = rotatepoint(q, centered_points');
            self.ordered_points = new_points' + self.pivot_node.location;
            
            self.shock_node.location = self.ordered_points(:, self.shock_index);
            self.control_arm_node.location = self.ordered_points(:, self.control_arm_index);
            if ~isnan(new_shock_point)
                if norm(self.shock_node.location - new_shock_point) > 0.0001
                    disp('SWITCHING ROTATION')
                    self.rotation_direction = -self.rotation_direction;
                    self.rotate(thetad * 2, NaN);
                end
            end
            
        end
        
        function update(self)
            pivot_point = self.pivot_node.location;
            shock_point = self.shock_node.location;
            control_arm_point = self.control_arm_node.location;
            
            self.plane = Plane(pivot_point, shock_point, control_arm_point);
            self.shock_lever= norm(shock_point - pivot_point);
            self.control_arm_lever = norm(control_arm_point - pivot_point);
            self.ordered_points = [pivot_point, shock_point, control_arm_point];
        end
        
    end
    
    methods (Access = private)
        function ordered_points = order_points(self)
            % Used to determine order of points to make a closed polygon
            % for plotting (only needed for 4+ points, so it isn't useful
            % unless we model an ARB eventually)
            pivot_point = self.pivot_node.location;
            shock_point = self.shock_node.location;
            control_arm_point = self.control_arm_node.location;
            
            all_points = [pivot_point, shock_point, control_arm_point];
            moveable_points = all_points(:, 2:end);
            avg_point = mean(all_points, 2);
            reference = mean(pivot_point - avg_point)*ones([1, 3]);
            direction = vecnorm(moveable_points - avg_point);
            unsigned_angles = acosd(dot(reference, direction));
            signed_angle_offset = (cross(reference, direction) < 0)*180;
            signed_angles = signed_angle_offset + unsigned_angles;
            [~, I] = sort(signed_angles);
            self.ordered_points = [pivot_point, moveable_points(:, I)];
            
            self.shock_index = find(I==1)+1;
            self.control_arm_index = find(I==2)+1; 
        end
    end
end