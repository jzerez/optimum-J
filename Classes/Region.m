classdef Region < handle
    
    properties
        sub_regions = [];
        num_regions;
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
            self.num_regions = nargin;
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
                self.sub_regions = centroid + permutations .* dists;
            else
                % If the only argument was a bounding box, accept it.
                assert(size(varargin{1}) == [3,8]);
                self.sub_regions = varargin{1};
            end
            self.generate_bounds();
        end
        
        function res = is_within_region(self, point)
            % Check to see if a point is within the region
            res = false;
            for region = 1:self.num_regions
                if point(1) <= self.max_x(:, :, region) && point(1) >= self.min_x(:, :, region)
                    if point(2) <= self.max_y(:, :, region) && point(2) >= self.min_y(:, :, region)
                        if point(3) <= self.max_z(:, :, region) && point(3) >= self.min_z(:, :, region)
                            res = true;
                            return;
                        end
                    end
                end
            end
        end
        
        function add_region(self, regions)
            for index = size(regions, 3)
                region = regions(:,:,index);
                self.sub_regions = cat(3, self.sub_regions, region);
            end
            self.num_regions = size(self.sub_regions, 3);
            self.generate_bounds();
        end
        
        function point = generate_starting_point(self)
            x = self.min_x + abs(self.max_x - self.min_x) * rand();
            y = self.min_y + abs(self.max_y - self.min_y) * rand();
            z = self.min_z + abs(self.max_z - self.min_z) * rand();
            point = [x; y; z];
        end
    end
    
    methods (Access = private)
        function generate_bounds(self)
            % Define bounds of regions
            self.max_x = max(self.sub_regions(1, :, :));
            self.max_y = max(self.sub_regions(2, :, :));
            self.max_z = max(self.sub_regions(3, :, :));
            self.min_x = min(self.sub_regions(1, :, :));
            self.min_y = min(self.sub_regions(2, :, :));
            self.min_z = min(self.sub_regions(3, :, :));
        end
    end
    
end
