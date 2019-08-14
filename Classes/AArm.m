classdef AArm < handle
    properties
        tip;
        endpoints;
        static_plane;
        action_plane;
        unit_directions;
        pivots;
        pivot_axis;
        effective_radius;
        effective_center;
        rotation_direction = 1;
        active;
        pushrod_mount;
        pushrod_plane;
        pushrod_center;
        pushrod_radius;
    end
   
    methods
        function self = AArm(varargin)
            % Constructor function. Takes 3 nodes as arguments for
            % "passive" AArm and 4 nodes as arguments for "active" AArm.
            
            % Should be in the format: tip_node, endpoint_node_1,
            % endpoint_node_2, pushrod_node
            self.tip = varargin{1};
            self.endpoints = [varargin{2}, varargin{3}];
            if nargin == 4
                self.active = true;
                self.pushrod_mount = varargin{4};
            else
                self.active = false;
                self.pushrod_mount = NaN;
            end
            
            self.update();
            
        end 

        function endpoints = get_endpoint_locations(self)
            endpoints = [self.endpoints(:,1).location, self.endpoints(:,2).location];
        end

        function self = rotate(self, thetad, new_tip_point)
            % center all points of interest around the first pivot point
            % for easier rotation
            centered_tip = self.tip.location - self.pivots(:, 1);
            centered_endpoints = [self.endpoints(1).location, self.endpoints(2).location] - self.pivots(:, 1);
            if self.active
                centered_pushrod_mount = self.pushrod_mount.location - self.pivots(:, 1);
            end
            
            angled = thetad / 2 * self.rotation_direction;
            real = cosd(angled);
            imag = sind(angled) * self.pivot_axis;
            q = quaternion(real, imag(1), imag(2), imag(3));
            
            new_tip = rotatepoint(q, centered_tip')' + self.pivots(:, 1);
            self.tip.location = new_tip;
            
            new_endpoints = rotatepoint(q, centered_endpoints')' + self.pivots(:, 1);
            self.endpoints(1).location = new_endpoints(:, 1);
            self.endpoints(2).location = new_endpoints(:, 2);
            
            if self.active
                new_pushrod_mount = rotatepoint(q, centered_pushrod_mount')' + self.pivots(:, 1);
                self.pushrod_mount.location = new_pushrod_mount;
            end
            
            % Check to see if we rotated in the right diretion and if not,
            % correct it
            if ~isnan(new_tip_point)
                if self.active
                    ref_point = self.pushrod_mount.location;
                else
                    ref_point = self.tip.location;
                end
                if norm(ref_point - new_tip_point) > 1e-5
                        self.rotation_direction = -self.rotation_direction;
                        self = self.rotate(thetad * 2, NaN);
                end
            end
        end
        
        function update(self)
            tip_loc = self.tip.location;
            self.static_plane = Plane(tip_loc, self.endpoints(1).location, self.endpoints(2).location);
            self.pivots = self.get_endpoint_locations();

            self.pivot_axis = unit(diff(self.pivots, 1, 2));
            self.effective_center = self.pivots(:, 2) - self.pivot_axis * dot(self.pivot_axis, self.pivots(:, 2) - self.tip.location);
            self.effective_radius = norm(self.effective_center - self.tip.location);

            assert(abs(dot(self.pivot_axis, unit(self.tip.location - self.effective_center))) < 1e-8);
            action_plane_point = self.tip.location + cross(self.pivot_axis, self.tip.location - self.effective_center);
            self.action_plane = Plane(self.effective_center, self.tip.location, action_plane_point);
            if self.active
                self.pushrod_plane = Plane(self.pushrod_mount.location, self.action_plane.normal);
                n = self.pushrod_plane.normal;
                x = dot(-self.pivots(:, 1) + self.pushrod_plane.position, n) / dot(self.pivot_axis, n);
                self.pushrod_center = self.pivots(:, 1) + (x*self.pivot_axis);
                self.pushrod_radius = norm(self.pushrod_center - self.pushrod_mount.location);
            end
            
        end
   end
end