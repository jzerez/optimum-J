classdef ActionGroup < handle
    properties
        static_rack;
        action_plane;
        colors = ['r', 'g', 'b', 'k', 'm', 'c'];
        plot_on = false;
        
        toelink_length;
        
        curr_rocker;
        curr_shock;
        curr_pushrod;
        curr_aca;
        curr_pca;
        curr_rack;
        curr_knuckle;
        
        front_suspension;
        
        % Static Characteristics (scalars)
        IC;
        RC;
        spindle_length;
        kingpin_angle;
        scrub_radius;
        anti_percentage;
        FVSA_length;
        SVSA_length;
        mechanical_trail;
        
        % Dynamic Characteristics (vectors/matrices)
        bump_steer;
        scrub;
        wheel_centers;
        cambers;
        
        
        front_view_plane = Plane([0; 0; 30.5], [0; 0; 1]);
        side_view_plane = Plane([25; 0; 0], [1; 0; 0]);
    end
   
    methods
        function self = ActionGroup(rocker, shock, pushrod, aca, pca, knuckle, rack)
            % Action group meant for sweeping the range of the suspension.
            self.static_rocker = rocker;
            self.static_shock = shock;
            self.static_pushrod = pushrod;
            self.static_aca = aca;
            self.static_pca = pca;
            self.static_knuckle = knuckle;
            self.static_rack = Rack(rack.location_node, rack.max_travel, rack.static_length);
            
            self.toelink_length = norm(knuckle.toe_point - rack.endpoint_location);
            
            self.curr_rocker = rocker;
            self.curr_shock = shock;
            self.curr_pushrod = pushrod;
            self.curr_aca = aca;
            self.curr_pca = pca;
            self.curr_knuckle = knuckle;
            self.curr_rack = rack; 
            self.action_plane = rocker.plane;
            
            assert(isequal(shock.outboard_point, rocker.shock_point));
            assert(isequal(knuckle.aca_point, aca.tip));
            assert(isequal(knuckle.pca_point, pca.tip));
%             assert(rocker.plane.is_in_plane(shock.inboard_node.location));
            
            self.IC = self.calc_instant_center();
            self.RC = self.calc_roll_center(self.IC);
        end
        
        function self = perform_sweep(self, num_steps, plot_on)
            self.plot_on = plot_on;
            static_lateral_pos = self.curr_aca.tip.location(1);
            shock_step_size = self.static_shock.total_travel / (num_steps - 1);
            shock_start_step = self.static_shock.total_travel / -2;
            rack_step_size = self.static_rack.max_travel / (num_steps - 1);
            rack_start_step = self.static_rack.max_travel / -2;
            shock_steps = linspace(shock_start_step, -shock_start_step, num_steps);
            rack_steps = linspace(rack_start_step, -rack_start_step, num_steps);
            
            self = self.take_shock_step(shock_start_step);
           
            cambers = zeros(num_steps);
            toes = zeros(num_steps);
            
            if plot_on
                plot_system_3d('c', self.curr_rocker, self.curr_shock, self.curr_aca, self.curr_pushrod, self.curr_pca, self.curr_knuckle);
            end

            [camber, toe] = self.perform_rack_sweep(rack_start_step, rack_step_size, num_steps);
            cambers(1, :) = camber;
            toes(1, :) = toe;
            
            for shock_index = 2:num_steps
                self = self.take_shock_step(shock_step_size);
                if plot_on
                    plot_system_3d('k', self.curr_rocker, self.curr_shock, self.curr_aca, self.curr_pushrod, self.curr_pca, self.curr_knuckle);
                    drawnow()
                end
                [camber, toe] = self.perform_rack_sweep(rack_start_step, rack_step_size, num_steps);
                toes(shock_index, :) = toe;
                cambers(shock_index, :) = camber;
            end

            if plot_on
                figure
                hold on
                
                imagesc(rack_steps, shock_steps, cambers)
                c = colorbar;
                ylabel(c, 'Camber (Degrees)')
                xlabel('Rack Displacement From Static (in)')
                ylabel('Shock Displacement From Static (in)')
                
                figure
                imagesc(rack_steps, shock_steps, toes)
                c = colorbar;
                ylabel(c, 'Toe (Degrees)')
                xlabel('Rack Displacement From Static (in)')
                ylabel('Shock Displacement From Static (in)')
                
            end
            self.take_shock_step(-shock_step_size * (num_steps - 1) - shock_start_step);
            self.reset_rack();
        end
        
        function self = take_shock_step(self, step)
            self.calc_rocker_movement(step);
            self.curr_aca = self.calc_xca_movement(self.curr_aca, self.curr_pushrod.inboard_node.location, self.curr_pushrod.length);
            self.curr_knuckle.aca_point = self.curr_aca.tip;
            
%             self.curr_pushrod.outboard_node.location = self.curr_aca.pushrod_mount.location;
            self.curr_pca = self.calc_xca_movement(self.curr_pca, self.curr_knuckle.aca_point.location, self.curr_knuckle.control_arm_dist);
            self.curr_knuckle.pca_point = self.curr_pca.tip;
        end
        
        function take_rack_step(self, step)
            self.curr_rack.calc_new_endpoint(step);
            self.calc_knuckle_rotation();
        end
        
        function calc_rocker_movement(self, step)
            prev_location = self.curr_shock.outboard_point;
            shock_radius = self.curr_shock.curr_length + step;
            shock_center = self.curr_shock.inboard_node.location;
            shock_center = self.action_plane.convert_to_planar_coor(shock_center);
            
            rocker_radius = self.curr_rocker.shock_lever;
            rocker_center = self.curr_rocker.pivot_point;
            rocker_center = self.action_plane.convert_to_planar_coor(rocker_center);
            
            [x, y] = circcirc(rocker_center(1), rocker_center(2), rocker_radius,...
                              shock_center(1), shock_center(2), shock_radius);
            p1 = [x(1); y(1)];
            p1 = self.action_plane.convert_to_global_coor(p1);
            p2 = [x(2); y(2)];
            p2 = self.action_plane.convert_to_global_coor(p2);
            new_location = self.find_closer_point(prev_location, p1, p2);
            
            new_rocker_pos = unit(new_location - self.curr_rocker.pivot_point);
            old_rocker_pos = unit(prev_location - self.curr_rocker.pivot_point);
            theta = -acosd(dot(old_rocker_pos, new_rocker_pos));

            self.curr_rocker.rotate(theta, new_location);
            self.curr_shock = self.curr_shock.new_outboard_point(new_location);
            self.curr_pushrod.inboard_node.location = self.curr_rocker.control_arm_point;
        end
        
        function new_xca = calc_xca_movement(self, xca, ref_point, ref_dist)
            if xca.active
                prev_location = xca.pushrod_mount.location;
                [int1, int2] = calc_sphere_circle_int(ref_point, ref_dist,...
                                              xca.pushrod_center, xca.pushrod_radius, xca.pushrod_plane);
                new_location = self.find_closer_point(prev_location, int1, int2);
                new_xca_pos = unit(new_location - xca.pushrod_center);
                old_xca_pos = unit(prev_location - xca.pushrod_center);
                assert(xca.pushrod_plane.is_in_plane(new_xca_pos + xca.pushrod_center));
                assert(xca.pushrod_plane.is_in_plane(old_xca_pos + xca.pushrod_center));
            else
                prev_location = xca.tip.location;
                [int1, int2] = calc_sphere_circle_int(ref_point, ref_dist,...
                                              xca.effective_center, xca.effective_radius, xca.action_plane);
                new_location = self.find_closer_point(prev_location, int1, int2);
                new_xca_pos = unit(new_location - xca.effective_center);
                old_xca_pos = unit(prev_location - xca.effective_center);
            end
            theta = -acosd(dot(old_xca_pos, new_xca_pos));
            new_xca = xca.rotate(theta, new_location);

            assert(abs(norm(new_location - ref_point) - ref_dist) < 1e-8);
        end
        
        function [toes, cambers] = perform_rack_sweep(self, rack_start_step, rack_step_size, num_steps)
            self.reset_rack();
            self.take_rack_step(rack_start_step);
            self.interference_detection(1);
            toes = zeros(1,num_steps);
            cambers = zeros(1,num_steps);
            self.curr_knuckle.update();
            [cambers(1), toes(1)] = self.curr_knuckle.calc_camber_and_toe();
            
            for index = 2:num_steps
                self.take_rack_step(rack_step_size);
                self.interference_detection(1);
                    
                self.curr_knuckle.update();
                [camber, toe] = self.curr_knuckle.calc_camber_and_toe();

                toes(index) = toe;
                cambers(index) = camber;
                if self.plot_on
                    plot_system_3d('g', self.curr_knuckle)
                end
            end
        end
        
        function point = find_closer_point(self, point_of_interest, p1, p2)
            dist1 = norm(point_of_interest - p1);
            dist2 = norm(point_of_interest - p2);
            
            if dist1 < dist2
                point = p1;
            else
                point = p2;
            end
        end
        
        function calc_knuckle_rotation(self)
            previous_location = self.curr_knuckle.toe_point;
            self.curr_knuckle.update_toe_plane();
            
            
            toe_center = self.curr_knuckle.toe_center;
            toe_radius = self.curr_knuckle.toe_radius;
            toe_plane = self.curr_knuckle.toe_plane;
            
            [p1, p2] = calc_sphere_circle_int(self.curr_rack.endpoint_location, self.toelink_length,...
                                              toe_center, toe_radius, toe_plane);
            
            new_location = self.find_closer_point(previous_location, p1, p2);

            self.curr_knuckle.toe_point = new_location;
        end
        
        function reset_rack(self)
            self.curr_rack.endpoint_location = self.static_rack.endpoint_location;
            self.calc_knuckle_rotation();
        end
        
        function res = interference_detection(self, debug)
            res = true;
            r = 0.3;
            % Check for toe-link aca interference
            if line_line_interference(self.curr_rack.endpoint_location, self.curr_knuckle.toe_point, r,...
                                      self.curr_aca.endpoints(1).location, self.curr_aca.tip.location, r)
                return;
            end
            % Check for toe-link aca interference (second arm)
            if line_line_interference(self.curr_rack.endpoint_location, self.curr_knuckle.toe_point, r,...
                                      self.curr_aca.endpoints(2).location, self.curr_aca.tip.location, r)
                return;
            end
            % Check for pushrod toelink interference
            if line_line_interference(self.curr_rack.endpoint_location, self.curr_knuckle.toe_point, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                return;
            end
%             
%             % Check for pushrod aca interference
%             if line_line_interference(self.curr_aca.endpoints(1).location, self.curr_aca.tip.location, r,...
%                                       self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
%                 return;
%             end
%             % Check for pushrod aca interference (second arm)
%             if line_line_interference(self.curr_aca.endpoints(2).location, self.curr_aca.tip.location, r,...
%                                       self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
%                 return;
%             end
            % Check for pushrod pca interference
            if line_line_interference(self.curr_pca.endpoints(1).location, self.curr_pca.tip.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                return;
            end
            % Check for pushrod pca interference (second arm)
            if line_line_interference(self.curr_pca.endpoints(2).location, self.curr_pca.tip.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                return;
            end
            
            for index = 1:length(self.curr_knuckle.wheel.radii)
                w = self.curr_knuckle.wheel;
                wr = w.radii(index);
                c = w.center + w.axial_dists(index) * w.axis;
                % Check for toe link interference
                if line_circle_interference(c, wr, self.curr_knuckle.wheel.plane,...
                                            self.curr_rack.endpoint_location, self.curr_knuckle.toe_point, r)
                    return;
                end
                if line_circle_interference(c, wr, self.curr_knuckle.wheel.plane,...
                                            self.curr_aca.endpoints(1).location, self.curr_aca.tip.location, r)
                    return;
                end
                if line_circle_interference(c, wr, self.curr_knuckle.wheel.plane,...
                                            self.curr_aca.endpoints(2).location, self.curr_aca.tip.location, r)
                    return;
                end
                if line_circle_interference(c, wr, self.curr_knuckle.wheel.plane,...
                                            self.curr_pca.endpoints(1).location, self.curr_pca.tip.location, r)
                    return;
                end
                if line_circle_interference(c, wr, self.curr_knuckle.wheel.plane,...
                                            self.curr_pca.endpoints(2).location, self.curr_pca.tip.location, r)
                    return;
                end
                if line_circle_interference(c, wr, self.curr_knuckle.wheel.plane,...
                                          self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                    return;
                end
            end
            
            res = false;
            if debug && res
                disp('INTERFERENCE DETECTED')
            end
        end
        
        function IC = calc_instant_center(self)
            [pca_point, pca_line] = self.curr_pca.static_plane.calc_plane_plane_int(self.front_view_plane);
            [aca_point, aca_line] = self.curr_aca.static_plane.calc_plane_plane_int(self.front_view_plane);
            eqns = [pca_line, aca_line, (aca_point - pca_point)];
            sols = rref(eqns);
            n = sols(1, 3);
            IC = pca_point - n * pca_line;
        end
        
        function RC = calc_roll_center(self, IC)
            cp = self.front_view_plane.project_into_plane(self.curr_knuckle.wheel.calc_contact_patch());
            v = unit(cp - IC);
            n = -IC(1) / v(1);
            RC = IC + n*v;
        end
    end
end