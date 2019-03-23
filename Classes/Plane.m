classdef Plane
   properties
        position;
        normal;
        i_hat;
        j_hat;
   end
   
   methods
       function self = Plane(varargin)
           % Constructor Function. Can define plane by 3 points, or two
           % vectors that are in plane
           if nargin > 2
               % Defined by 3 Points
               self.position = varargin{1};
               v1 = varargin{2} - varargin{1};
               v2 = varargin{3} - varargin{1};
               normal = cross(v1, v2);
               self.normal = normal / norm(normal);
           elseif isa(varargin{1}, 'Line') && isa(varargin{2}, 'Line')
                % Defined by 2 Planar Line Segments
                line1 = varargin{1};
                line2 = varargin{2};
                self.position = line1.position;
                normal = cross(line1.axis, line2.axis);
                self.normal = normal / norm(normal);
           else
               % Defined by a normal vector and a position
               self.position = varargin{1};
               self.normal = varargin{2};
           end
           if isequal(self.normal, [1;0;0])
                ibase = [0; 1; 0];
           else
                ibase = [1; 0; 0];
           end
           i = self.project_into_plane(self.position + ibase) - self.position;
           self.i_hat = i / norm(i);
           self.j_hat = cross(self.normal, self.i_hat);
           assert(self.is_in_plane([self.i_hat+self.j_hat+self.position]))
       end
       
       function res = is_in_plane(self, point)
            v = unit(point - self.position);
            dist = abs(dot(v, self.normal));
            if dist < 1e-8
                res = true;
                return
            end
            res = false;
            disp('dist to point is:')
            disp(dist)
       end
       
       function new_point = project_into_plane(self, point)
            norm_dist = self.calc_norm_dist(point);
            new_point = point - (norm_dist*self.normal);
            assert(self.is_in_plane(new_point))
       end
       
       function r2_points = convert_to_planar_coor(self, points)
            r2_points = zeros([2, size(points, 2)]);
            for index = 1:size(points,2)
                v = points(:, index) - self.position;
                i_mag = dot(v, self.i_hat);
                j_mag = dot(v, self.j_hat);
                r2_points(:, index) = [i_mag;j_mag];
            end
       end
       
       function r3_points = convert_to_global_coor(self, points)
            r3_points = zeros([3, size(points, 2)]);
            for index = 1:size(points,2)
                p_2d = points(:, index);
                p_3d = p_2d(1)*self.i_hat + p_2d(2)*self.j_hat;
                %assert(self.is_in_plane(p_3d + self.position))
                r3_points(:, index) = p_3d + self.position;
            end
       end
       
       function norm_dist = calc_norm_dist(self, point)
            dist = point - self.position;
            norm_dist = dot(dist, self.normal);
       end
       
   end
end