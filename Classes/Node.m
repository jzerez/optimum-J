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
                if ~isnan(location)
                    self.region = Region(location, varargin{2});
                else
                    self.region = Region(varargin{2});
                end
            end
            
            % If the user doesn't specify an initial starting point, pick
            % one
            if isnan(location)
                self.location = self.region.generate_starting_point();
            else
                assert(self.region.is_within_region(location));
                self.location = location;
            end
            
            
        end
    end
end