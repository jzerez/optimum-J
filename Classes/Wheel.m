classdef Wheel < handle
    properties
        center;         % Wheel center
        axis;           % Axis of wheel
        axis_point;     % An arbitrary point on the wheel axis
        radii;          % Radii describing contours of the wheel
        axial_dists;    % axial dists of each radii from the wheel center
        static_camber;  % static camber. Negative is negative camber (lean in)
        static_toe;     % static toe. Positive is toe in
        contact_patch;  % Location of the contact patch 
        
        plane;          % Wheel plane. Normal to axis, coincident with center
    end
    
    methods
        function self = Wheel(static_camber, static_toe, wheel_center, radii, axial_dists)
            self.static_camber = static_camber;
            self.static_toe = static_toe;
            assert(numel(radii) == numel(axial_dists));
            self.radii = radii;
            self.axial_dists = axial_dists;
            self.center = wheel_center;
            y = sind(static_camber);
            h = cosd(static_camber);
            x = cosd(static_toe) / h;
            z = sind(static_toe) / h;
            
            % Axis perpendicular to the plane of the wheel
            self.axis = unit([x; y; z]);
            self.axis_point = self.center + self.axis;
            
            self.plane = Plane(self.center, self.axis);
            self.update();
        end
        
        function update(self)
            vec = [0; -1; 0] + self.center;
            self.contact_patch = unit(self.plane.project_into_plane(vec) - self.center) * self.radii(1) + self.center;
        end
        
    end
end