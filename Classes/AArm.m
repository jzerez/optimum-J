classdef AArm
    properties
        tip;
        endpoints;
        static_plane;
        action_plane;
        tab_offset = 1.5;        %in
        unit_directions;
        pivots;
        pivot_axis;
        effective_radius;
        effective_center;
        rotation_direction = 1;
    end
   
    methods
        function self = AArm(tip_point, node1, node2)
            % Constructor function. Takes 3 nodes as arguments

            self.tip = tip_point;
            self.endpoints = [node1, node2];
            self = self.update();
            
        end 

        function endpoints = get_endpoint_locations(self)
            endpoints = [self.endpoints(:,1).location, self.endpoints(:,2).location];
        end

        function self = rotate(self, thetad, new_tip_point)
            centered_tip = self.tip.location - self.pivots(:, 1);
            centered_endpoints = [self.endpoints(1).location, self.endpoints(2).location] - self.pivots(:, 1);
            angled = thetad / 2 * self.rotation_direction;
            real = cosd(angled);
            imag = sind(angled) * self.pivot_axis;
            q = quaternion(real, imag(1), imag(2), imag(3));
            new_tip = rotatepoint(q, centered_tip')' + self.pivots(:, 1);
            new_endpoints = rotatepoint(q, centered_endpoints')' + self.pivots(:, 1);
            self.tip.location = new_tip;
            self.endpoints(1).location = new_endpoints(:, 1);
            self.endpoints(2).location = new_endpoints(:, 2);
            
            if ~isnan(new_tip_point)
                if norm(self.tip.location - new_tip_point) > 1e-5
                    self.rotation_direction = -self.rotation_direction;
                    self = self.rotate(thetad * 2, NaN);
                end
            end
        end
        
        function self = update(self)
            tip_loc = self.tip.location;
            endpoints_loc = [self.endpoints(1).location, self.endpoints(2).location];
            self.static_plane = Plane(tip_loc, self.endpoints(1).location, self.endpoints(2).location);
            directions = tip_loc - endpoints_loc;
            self.unit_directions = directions ./ vecnorm(directions);
            self.pivots = endpoints_loc + (self.unit_directions * self.tab_offset);
            self.pivot_axis = unit(diff(self.pivots, 1, 2));
            self.effective_center = self.pivots(:, 2) - self.pivot_axis * dot(self.pivot_axis, self.pivots(:, 2) - self.tip.location);
            self.effective_radius = norm(self.effective_center - self.tip.location);

            assert(abs(dot(self.pivot_axis, unit(self.tip.location - self.effective_center))) < 1e-8);
            action_plane_point = self.tip.location + cross(self.pivot_axis, self.tip.location - self.effective_center);
            self.action_plane = Plane(self.effective_center, self.tip.location, action_plane_point);
        end
   end
end