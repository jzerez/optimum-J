classdef Knuckle
    properties
        uca_point;
        lca_point;
        toe_point;
        axis;
        static_camber;
        camber_offset;
        static_toe;
        toe_radius;
        toe_height;
        toe_center;
        control_arm_dist;
        lca_actuated;
        toe_plane;
        
    end
    
    methods
        function self = Knuckle(uca_point, lca_point, toe_point, static_camber, static_toe, lca_actuated)
            self.uca_point = uca_point;
            self.lca_point = lca_point;
            self.toe_point = toe_point;
            
            self.axis = unit(uca_point.location - lca_point.location);
            self.static_camber = static_camber;
            self.camber_offset = static_camber - atan2d(self.axis(1), self.axis(2));
            self.static_toe = static_toe;
            toe_to_lca = (static_toe - self.uca_point.location);
            self.toe_height = dot(toe_to_lca, self.axis);
            self.toe_radius = norm(toe_to_lca - self.toe_height*self.axis + self.lca_point.location);
 
            self.control_arm_dist = norm(uca_point.location - lca_point.location);
            self.lca_actuated = lca_actuated;
            
            self.toe_plane = self.calc_toe_plane();
        end
        
        function res = valid_length(self)
            dist = norm(self.uca_point.location - self.lca_point.location);
            res = (abs(dist - self.control_arm_dist) < 1e-8);
        end
        
        function camber = calc_camber(self)
            self.axis = unit(self.uca_point.location - self.lca_point.location);
            camber = atan2d(self.axis(1), self.axis(2)) + self.camber_offset;
        end
        
        function plane = calc_toe_plane(self)
            self.toe_center = self.toe_height * self.axis + self.lca_point.location;
            plane = Plane(self.toe_center, self.axis);
        end
        
    end
end