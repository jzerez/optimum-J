classdef Shock
    properties
        inboard_node;
        outboard_node;
        action_plane;
        static_length = mm_to_in(250);
        curr_length;
        total_travel = mm_to_in(50);
    end
    
    methods
        function self = Shock(inboard_node, outboard_node, action_plane)
            self.action_plane = action_plane;
            self.inboard_node = inboard_node;
            self.outboard_node = outboard_node;
            self.curr_length = self.calc_length();
        end
        
        function self = new_outboard_point(self, point)
            self.outboard_node.location = point;
            self.curr_length = self.calc_length();
        end
        
        function length = calc_length(self)
            length = norm(self.outboard_node.location - self.inboard_node.location);
            %assert(length - self.static_length);
        end
    end
end