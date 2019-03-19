classdef Rack
    properties
        location_node;
        max_travel;
        static_length;
        endpoint_location;
    end

    methods
        function self = Rack(location_node, max_travel, static_length)
            self.location_node = location_node;
            self.max_travel = max_travel;
            self.static_length = static_length;
            self.endpoint_location = location_node.location + (static_length / 2) * [1;0;0];
            % Rack should be centered left right in the car
            %assert(location_node.region.max_x == location_node.region.min_x);
        end
        function self = calc_new_endpoint(self, displacement)
            % weak assertion. Needs to check against static length, not max
            % travel.
%             assert(abs(displacement) <= self.max_travel /2 )
            self.endpoint_location = self.endpoint_location + displacement * [1;0;0];
        end
        
        function self = reset_endpoint(self)
            self.endpoint_location = self.location_node.location + (self.static_length / 2) * [1;0;0];
        end
    end
end