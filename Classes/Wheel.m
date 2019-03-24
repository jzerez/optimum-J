classdef Wheel
    properties
        center;
        axis;
        axis_point;
        radii;
        axial_dists;
        static_camber;
        static_toe;
        
        plane;
    end
    
    methods
        function self = Wheel(static_camber, static_toe, wheel_center, radii, axial_dists)
            self.static_camber = static_camber;
            self.static_toe = static_toe;
            assert(numel(radii) == numel(axial_dists));
            self.radii = [wheel_center(2), radii];
            self.axial_dists = [0, axial_dists];
            self.center = wheel_center;
            y = sind(static_camber);
            h = cosd(static_camber);
            x = cosd(static_toe) / h;
            z = sind(static_toe) / h;
            self.axis = unit([x; y; z]);
            self.axis_point = self.center + self.axis;
            
            self.plane = Plane(self.center, self.axis);
        end
        
    end
end