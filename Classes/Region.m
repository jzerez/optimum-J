classdef Region < handle
    
    properties
        max_x;
        max_y;
        max_z;
        min_x;
        min_y;
        min_z;
    end
    
    methods
        function self = Region(varargin)
            %Constructor Function, takes any number of bounding box
            %arguments. A bounding box is a 3x8 array of XYZ points that
            %define the corners of a region.
            if nargin > 1
                % Generate the bounding box based on two arguments:
                    % Centroid - XYZ vector that is the center of the box
                    % Size - XYZ vector that describes distance from
                    % centroid to bounding box walls in 3 dimensions
                centroid = varargin{1};
                dists = varargin{2};
                permutations = [-1, -1, -1, -1,  1,  1,  1,  1;...
                                -1, -1,  1,  1, -1, -1,  1,  1;...
                                -1,  1, -1,  1, -1,  1, -1,  1;];
                corner_coor = centroid + permutations .* dists;
                self.generate_bounds(corner_coor);
            else
                % If the only argument are limits:
                % [xmin, ymin, zmin;...
                %  xmax, ymax, zmax];
                lims = varargin{1};
                assert(isequal(size(lims), [2,3]));
                
                self.min_x = lims(1);
                self.max_x = lims(2);
                self.min_y = lims(3);
                self.max_y = lims(4);
                self.min_z = lims(5);
                self.max_z = lims(6);
                
            end
            
        end
        
        function res = is_within_region(self, point)
            % Check to see if a point is within the region
            res = false;
            if point(1) <= self.max_x && point(1) >= self.min_x
                if point(2) <= self.max_y && point(2) >= self.min_y
                    if point(3) <= self.max_z && point(3) >= self.min_z
                        res = true;
                        return;
                    end
                end
            end
        end
        
        function point = generate_starting_point(self)
            x = self.min_x + abs(self.max_x - self.min_x) * rand();
            y = self.min_y + abs(self.max_y - self.min_y) * rand();
            z = self.min_z + abs(self.max_z - self.min_z) * rand();
            point = [x; y; z];
            assert(self.is_within_region(point))
        end
    end
    
    methods (Access = private)
        function generate_bounds(self, corner_coor)
            % Define bounds of regions
            self.max_x = max(corner_coor(1, :, :));
            self.max_y = max(corner_coor(2, :, :));
            self.max_z = max(corner_coor(3, :, :));
            self.min_x = min(corner_coor(1, :, :));
            self.min_y = min(corner_coor(2, :, :));
            self.min_z = min(corner_coor(3, :, :));
        end
    end
    
end
