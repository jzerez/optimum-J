classdef Knuckle
    properties
        uca_point;
        lca_point;
        toe_point;
        axis;
        toe_radius;
        toe_height;
        toe_center;
        control_arm_dist;
        lca_actuated;
        toe_plane;
        action_plane; 
    end
    
    methods
        function self = Knuckle(uca_point, lca_point, toe_point, lca_actuated)
            self.uca_point = uca_point;
            self.lca_point = lca_point;
            self.toe_point = toe_point;
            
            self.axis = unit(uca_point.location - lca_point.location);
            
            toe_to_lca = (toe_point - self.lca_point.location);
            self.toe_height = dot(toe_to_lca, self.axis);
            self.toe_center = self.toe_height * self.axis + self.lca_point.location;
            self.toe_radius = norm(toe_to_lca - (self.toe_height*self.axis));
            self.toe_plane = Plane(self.toe_center, self.axis);
            
            self.control_arm_dist = norm(uca_point.location - lca_point.location);
            self.lca_actuated = lca_actuated;
            
            self.action_plane = Plane(uca_point.location, lca_point.location, toe_point);
        end
        
        function res = valid_length(self)
            dist = norm(self.uca_point.location - self.lca_point.location);
            res = (abs(dist - self.control_arm_dist) < 1e-8);
        end
        
        function camber = calc_camber(self)
            self.axis = unit(self.uca_point.location - self.lca_point.location);
            camber = atan2d(self.axis(1), self.axis(2)) + self.camber_offset;
        end
        
        function self = update_toe_plane(self)
            self.axis = unit(self.uca_point.location - self.lca_point.location);
            self.toe_center = self.toe_height * self.axis + self.lca_point.location;
            self.toe_plane = Plane(self.toe_center, self.axis);
        end
        
        function theta = calc_signed_steering_angle_raw(self)
            toe_lever = unit(self.toe_point - self.toe_center);
            forward_v = unit(self.toe_plane.project_into_plane([0;0;1] + self.toe_center) - self.toe_center);
            unsigned_toe_offset = acosd(dot(toe_lever, forward_v));
            sign = -dot(unit(cross(self.toe_center+forward_v, self.toe_center+toe_lever)), self.axis);
            if sign > 0
                direction = 1;
            else
                direction = -1;
            end
            theta = unsigned_toe_offset * direction;
%             quiver3(self.toe_center(1), self.toe_center(2), self.toe_center(3), forward_v(1), forward_v(2), forward_v(3), 'b')
%             quiver3(self.toe_center(1), self.toe_center(2), self.toe_center(3), toe_lever(1), toe_lever(2), toe_lever(3), 'r');
        end
        
        function self = update(self)
            self.action_plane = Plane(self.uca_point.location, self.lca_point.location, self.toe_point);
        end
        
        function [camber, toe] = calc_camber_and_toe(self)
            self = self.update();
            n = self.action_plane.normal;
            if n(1) > 0
                n = n * -1;
            end
            
            if n(2) > 0
                camber_direction = 1;
            else
                camber_direction = -1;
            end
            
            if n(3) > 0
                toe_direction = 1;
            else
                toe_direction = -1;
            end
            camber_proj_v = unit([n(1); n(2)]);
            toe_proj_v = unit([n(1); n(3)]);
            x = [-1;0];
            camber = acosd(dot(x, camber_proj_v)) * camber_direction;
            toe = acosd(dot(x, toe_proj_v)) * toe_direction;

            
        end
        
    end
end