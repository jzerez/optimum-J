classdef Line
    properties
        length;
        axis;
        inboard_point;
        outboard_point
    end
    
    methods
        function self = Line(inboard_point, outboard_point)
            axis = inboard_point - outboard_point;
            self.length = norm(axis);
            self.axis = unit(axis);
            assert(self.length > 0);
            self.inboard_point = inboard_point;
            self.outboard_point = outboard_point;
        end
        
        function res = valid_length(self)
            curr_length = norm(self.inboard_point - self.outboard_point);
            if abs(curr_length - self.length) < 1e-8
                res = true;
            else
                res = false;
            end
        end
    end
end