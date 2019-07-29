classdef Node < handle
    properties
        location;
        region;
    end
    
    methods
        function self = Node(varargin)
            assert(nargin == 2)

            location = varargin{1};
            if isa(varargin{2}, 'Region')
                % If the second argument is a region object
                self.region = varargin{2};
            else
                % if the second argument is a distance vector, create a
                % region object.
                self.region = Region(location, varargin{2});
            end
            assert(self.region.is_within_region(location));
            self.location = location;
        end
    end
end