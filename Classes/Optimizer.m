classdef Optimizer
    properties
        suspension;
        desired_static_char;
        static_char_weights;
        desired_dyn_char;
        dyn_char_weights;
        ds = 0.001;         % differential step size to calc gradient (inches)
        step_size = 0.125;   % step size between states (inches)
        
        static_locations;
        gradients;
        fitnesses;
    end
    
    methods (Access = public)
        function self = Optimizer(suspension, max_steps, desired_static_char, static_char_weights)
            self.suspension = suspension;
            num_nodes = length(self.suspension.node_list);
            self.static_char_weights = static_char_weights;
            self.desired_static_char = desired_static_char;
            
            
            self.get_node_locations()
            self.get_inequality_constraints()
            self.static_locations = zeros([3, num_nodes, max_steps+1]);
            self.fitnesses = zeros([1, max_steps+1]);
            self.gradients = zeros([num_nodes, max_steps+1]);
        end
        
        function optimize(self)
            num_sweeps = 3;
            self.static_locations(:, :, 1) = self.get_node_locations();
            self.fitnesses(1) = self.get_fitness(num_sweeps);
            self.gradients(:, 1) = self.calc_gradient(num_sweeps);
            
            for step = 1:max_steps
                
                curr_fitness = self.get_fitness(num_sweeps);
                gradient = self.calc_gradient(num_sweeps);
                take_step(gradient);
                
                new_fitness = self.get_fitness(num_sweeps);
                if new_fitness < curr_fitness
                    % Accept Move and store info
                    self.static_locations(:, :, step + 1) = self.get_node_locations();
                    self.fitnesses(step + 1) = new_fitness;
                    self.gradients(:, step + 1) = self.calc_gradient(num_sweeps);
                else
                    % Simulated Annealing logic goes here
                    if accept_simulated_annealing
                        % If we accept resutl from simulated annealing
                    else
                        % end simulation
                        fprintf('Local Minimum reached, fitness is %i \n', new_fitness);
                        return
                    end
                end
            end
        end
    end
    
    methods (Access = private)
        function gradient = calc_gradient(self, num_sweeps)
            curr_fitness = get_fitness(num_sweeps);
            nodes = self.suspension.node_list;
            gradient = zeros([1, numel(nodes)]);
            index = 1;
            % Sweep all nodes
            for node = nodes
                % Sweep for each direction
                for direction = 1:3
                    node.location(direction) = node.location(direction) + self.ds;
                    gradient(index) = get_fitness(num_sweeps) - curr_fitness;
                    index = index + 1;
                    node.location(direction) = node.location(direction) - self.ds;
                end
            end
        end
        
        function take_step(self, gradient)
            steps = gradient * norm(gradient) / self.step_size;
            nodes = self.suspension.node_list;
            for index = 1:length(steps)
                node = nodes(index);
                step = steps(index);
                node.location = node.location + step;
            end
        end
        
        function fitness = get_fitness(self, num_sweeps)
            [static_char, dyn_char] = self.suspension.perform_sweep(num_sweeps, 0);
            fitness = calc_fitness(static_char, self.desired_static_char, self.static_char_weights);
        end
        
        function locations = get_node_locations(self)
            nodes = self.suspension.node_list;
            locations = zeros([3, length(nodes)]);
            for index = 1:length(nodes)
                locations(:,index) = nodes(index).location;
            end
            locations = reshape(locations, [numel(locations), 1]);
        end
        
        function update_locations(self, new_locations)
            new_locations = reshape(new_locations, [3, numel(new_locations)/3]);
            nodes = self.suspension.node_list;
            for index = 1:length(nodes)
                nodes(index).location = new_locations(:, index);
            end
        end
        
        function A = get_inequality_constraints(self)
            nodes = self.suspension.node_list;
            A = zeros([1, 3*length(nodes) + 1]);
            region_lims = {'max_x', 'min_x', 'max_y', 'min_y', 'max_z',  'min_z'};
            for index = 1:length(nodes)
                for dimension = 1:3
                    row = size(A, 1) + 1;
                    col = (index - 1) * 3 + dimension;
                    limit_index = (index - 1) * 2 + 1;
                    A(row, col) = 1; 
                    A(row + 1, col) = -1;
                    A(row, end) = nodes(index).region.(region_lims(limit_index));
                    A(row + 1, end) = -nodes(index).region.(region_lims(limit_index + 1));
                end
            end
        end
    end
        
        
end