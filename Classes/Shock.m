classdef Shock
    properties
        inboard_node;
        outboard_point;
        action_plane;
        static_length;
        curr_length;
        total_travel = mm_to_in(50);
    end
    
    methods
        function self = Shock(inboard_node, outboard_point, action_plane)
            self.static_length = mm_to_in(250);
            self.action_plane = action_plane;
            self.inboard_node = inboard_node;
            self.outboard_point = outboard_point;
            self.curr_length = self.calc_length();
        end
        
        function self = new_outboard_point(self, point)
            self.outboard_point = point;
            self.curr_length = self.calc_length();
        end
        
        function length = calc_length(self)
            length = norm(self.outboard_point - self.inboard_node.location);
            %assert(length - self.static_length);
        end
    end
end