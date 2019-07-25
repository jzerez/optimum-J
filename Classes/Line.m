classdef Line < handle
    properties
        length;
        axis;
        inboard_node;
        outboard_node
    end
    
    methods
        function self = Line(inboard_node, outboard_node)
            self.inboard_node = inboard_node;
            self.outboard_node = outboard_node;
            self.update();
        end
        
        function res = valid_length(self)
            curr_length = norm(self.inboard_node.location - self.outboard_node.location);
            if abs(curr_length - self.length) < 1e-8
                res = true;
            else
                res = false;
            end
        end
        
        function update(self)
            axis = self.inboard_node.location - self.outboard_node.location;
            self.length = norm(axis);
            self.axis = unit(axis);
            assert(self.length > 0);
        end
    end
end