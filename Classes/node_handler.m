classdef node_handler < handle
    properties
        value;
    end
    
    methods
        function self = node_handler(value)
            self.value = value;
            
        end
        
        function task(self, x)
            self.value = self.value + x;
        end
    end
    
end