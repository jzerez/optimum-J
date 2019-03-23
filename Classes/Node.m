classdef Node < handle
    properties
        location;
        region;
    end
    
    methods
        function self = Node(location, region)
            assert(region.is_within_region(location));
            self.location = location;
            self.region = region;
        end
    end
end