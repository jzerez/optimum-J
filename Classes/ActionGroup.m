% Action group meant for sweeping the range of the suspension.
classdef ActionGroup < handle
    
    properties
        % Plane of the rocker
        action_plane;
        colors = ['r', 'g', 'b', 'k', 'm', 'c'];
        
        plot_on = false;
        
        % Length of toelink [in]
        toelink_length;
        
        % Rocker object
        curr_rocker;
        
        % Shock object
        curr_shock;
        
        % Pushrod (Line object)
        curr_pushrod;
        
        % Active Control Arm (attached to the pushrod) (AArm Object)
        curr_aca;
        
        % Passive Control Arm (not attached to pushrod) (AArm Object)
        curr_pca;
        
        % Rack object
        curr_rack;
        
        % Knuckle object
        curr_knuckle;
        
        % Car's wheelbase [in]
        wheelbase = 61;
        
        % Center of Gravity Height [in]
        cgh = 9;
        
        % Type of suspension ['front' or 'rear']
        suspension_type;
        
        % Static Characteristics (scalars)
        static_char = struct('RCH', NaN,...
                             'spindle_length', NaN,...
                             'kingpin_angle', NaN,...
                             'scrub_radius', NaN,...
                             'anti_percentage', NaN,...
                             'FVSA', NaN,...
                             'SVSA', NaN,...
                             'mechanical_trail', NaN,...
                             'interference', false);
        
        % Dynamic Characteristics (vectors/matrices)
        dyn_char = struct('contact_patches', NaN,...
                          'cambers', NaN,...
                          'toes', NaN);
        
        front_view_plane;
        side_view_plane;
        
        % Front brake bias
        front_brake_percentage = 0.4;
        
        % Radius of the suspension members [in]
        suspension_member_radius = 0.25;
    end
   
    methods
        function self = ActionGroup(rocker, shock, pushrod, aca, pca, knuckle, rack, suspension_type)
            % Action group meant for sweeping the range of the suspension.
            assert(strcmp(suspension_type, 'front') || strcmp(suspension_type, 'rear'), 'Valid suspension types are "front" and "rear"');
            self.suspension_type = suspension_type;
            
            self.toelink_length = norm(knuckle.toe_node.location - rack.endpoint_location);
            
            self.curr_rocker = rocker;
            self.curr_shock = shock;
            self.curr_pushrod = pushrod;
            self.curr_aca = aca;
            self.curr_pca = pca;
            self.curr_knuckle = knuckle;
            self.curr_rack = rack; 
            self.action_plane = Plane(shock.inboard_node.location, pushrod.outboard_node.location, rocker.pivot_node.location);
            if self.action_plane.normal(3) < 1
                self.action_plane.normal = -self.action_plane.normal;
                self.action_plane.update();
            end
            
%             assert(rocker.plane.is_in_plane(shock.inboard_node.location));
            
            c = self.curr_knuckle.wheel.center;
            self.front_view_plane = Plane([0; 0; c(3)], [0; 0; 1]);
            self.side_view_plane = Plane([c(1); 0; 0], [1; 0; 0]);
            
            self.calc_static_char();
            
        end
        
        function [static_char, dyn_char] = perform_sweep(self, num_steps, plot_on)
            % Sweeps the suspension through its range of motion (Shock + steering if applicable)
            % Arguments: 
            %   num_steps (int): Number of locations to caclulate between
            %                    suspension position extremes
            %   plot_on  (bool): Whether to plot the suspension or not
            % 
            % Returns:
            %   static_char (struct): Contains the static characteristics
            %                         for the given geometry
            %   dyn_char    (struct): Contains the dynamic characteristics
            %                         for the given geometry (vectors)
            
            fprintf('STARTING SWEEP\n')
            self.plot_on = plot_on;
            % Plots initial position of suspension
            if plot_on
                clf;
                hold on;
                plot_system_3d('b', self.curr_knuckle);
                plot_system_3d('r', self.curr_aca, self.curr_pca);
                plot_system_3d('k', self.curr_rack, self.curr_rocker, self.curr_shock, self.curr_pushrod);
            end
            
            % Define the shock and rack positions to sweep over
            %
            % NOTE: This assumes that the shock's neutral displacement is
            % half of its travel
            shock_step_size = self.curr_shock.total_travel / (num_steps - 1);
            shock_start_step = self.curr_shock.total_travel / -2;
            rack_step_size = self.curr_rack.max_travel / (num_steps - 1);
            rack_start_step = self.curr_rack.max_travel / -2;
            shock_steps = linspace(shock_start_step, -shock_start_step, num_steps);
            rack_steps = linspace(rack_start_step, -rack_start_step, num_steps);
            
            % move the shock to the starting position
            self.take_shock_step(shock_start_step);
           
            % Initialize vectors to store results
            cambers = zeros(num_steps);
            toes = zeros(num_steps);
            contact_patches = zeros([num_steps, num_steps, 3]);
            
            if plot_on
                plot_system_3d('c', self.curr_rocker, self.curr_shock, self.curr_aca, self.curr_pushrod, self.curr_pca, self.curr_knuckle);
            end
            
            % Sweeps through the rack positions 
            [camber, toe, cp] = self.perform_rack_sweep(rack_start_step, rack_step_size, num_steps);
            
            % Save results from the first rack sweep
            cambers(1, :) = camber;
            toes(1, :) = toe;
            contact_patches(1, :, :) = cp;
            
            % Sweep through the rest of the shock positions, doing a rack
            % sweep at each position. 
            for shock_index = 2:num_steps
                self.take_shock_step(shock_step_size);
                if plot_on
                    plot_system_3d('k', self.curr_rocker, self.curr_shock, self.curr_aca, self.curr_pushrod, self.curr_pca, self.curr_knuckle);
                    drawnow()
                end
                [camber, toe, cp] = self.perform_rack_sweep(rack_start_step, rack_step_size, num_steps);
                toes(shock_index, :) = toe;
                cambers(shock_index, :) = camber;
                contact_patches(shock_index, :, :) = cp;
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
                hold on;
                imagesc(rack_steps, shock_steps, toes)
                c = colorbar;
                ylabel(c, 'Toe (Degrees)')
                xlabel('Rack Displacement From Static (in)')
                ylabel('Shock Displacement From Static (in)')
                
            end
            % Return to neutral position
            self.take_shock_step(-shock_step_size * (num_steps - 1) - shock_start_step);
            
            % Resets the rack to neutral position
            self.reset_rack();
            % Resets the knuckle toe plane (to correctly calc static char)
            self.curr_knuckle.update_toe_plane();
            self.calc_static_char();
            static_char = self.static_char;
            
            % Save dynamic characterisitcs
            self.dyn_char.toes = toes;
            self.dyn_char.cambers = cambers;
            self.dyn_char.contact_patches = contact_patches;
            dyn_char = self.dyn_char;


            [c, t] = self.curr_knuckle.calc_camber_and_toe();
            if strcmp(self.suspension_type, 'front')
                cp = [25; 8.98; 30.5];
            else
                cp = [25; 8.98; -30.5];
            end
            
            if abs(c - self.wheel.static_camber) > 1e-3 || abs(t-self.wheel.static_toe) > 1e-3 || norm(self.curr_knuckle.wheel.center - cp) > 1e-3
                disp('did not reset toe/camber correctly :(')
            end
        end
        
        function take_shock_step(self, step)
            % Changes shock displacement by a specified amount and recalculates suspension positions
            %
            % Arguments:
            %   step (float): Change in shock length (inches)
            %
            % Returns:
            %   N/A
            
            % Calculate how the rocker moves as a result
            self.calc_rocker_movement(step);
            
            % Calculate how the active control arm moves
            self.curr_aca = self.calc_xca_movement(self.curr_aca, self.curr_pushrod.inboard_node.location, self.curr_pushrod.length);
            % Update the knuckle's active control arm node
            self.curr_knuckle.aca_node = self.curr_aca.tip;
            
            % Update pushrod outboard node location
            self.curr_pushrod.outboard_node.location = self.curr_aca.pushrod_mount.location;
            
            % Calculate how the passive control arm moves
            self.curr_pca = self.calc_xca_movement(self.curr_pca, self.curr_knuckle.aca_node.location, self.curr_knuckle.a_arm_dist);
            % Update the knuckle's passive control arm node
            self.curr_knuckle.pca_node = self.curr_pca.tip;
        end
        
        function take_rack_step(self, step)
            % Changes rack displacement by a specified amount and recalculates suspension positions
            %
            % Arguments:
            %   step (float): Change in rack displacement (inches)
            %
            % Returns:
            %   N/A
            self.curr_rack.calc_new_endpoint(step);
            self.calc_knuckle_rotation();
        end
        
        function calc_rocker_movement(self, step)
            % Calculates how much the rocker rotates for a given change in shock displacement
            %
            % Arguments:
            %   step (float): Change in shock displacement (inches)
            %
            % Returns:
            %   N/A
            
            % Treat the shock like a circle that lives in the action plane
            % of the rocker. The circle's center is the inboard mounting
            % location and the radius is the shock's length
            prev_location = self.curr_shock.outboard_node.location;
            shock_radius = self.curr_shock.curr_length + step;
            shock_center = self.curr_shock.inboard_node.location;
            shock_center = self.action_plane.convert_to_planar_coor(shock_center);
            
            % Treat the rocker like a circle that lives in the action plane
            % of the rocker. The circle's center is the inboard mounting
            % location and the radius is the shock's length
            rocker_radius = self.curr_rocker.shock_lever;
            rocker_center = self.curr_rocker.pivot_node.location;
            rocker_center = self.action_plane.convert_to_planar_coor(rocker_center);
            
            % Find the possible intersection points
            [x, y] = circcirc(rocker_center(1), rocker_center(2), rocker_radius,...
                              shock_center(1), shock_center(2), shock_radius);
            
            % find the two intersection points and convert back into global
            % coordinates
            p1 = [x(1); y(1)];
            p1 = self.action_plane.convert_to_global_coor(p1);
            p2 = [x(2); y(2)];
            p2 = self.action_plane.convert_to_global_coor(p2);
            % choose the point closer to the original point
            new_location = find_closer_point(prev_location, p1, p2);
            
            % update rocker position
            new_rocker_pos = unit(new_location - self.curr_rocker.pivot_node.location);
            old_rocker_pos = unit(prev_location - self.curr_rocker.pivot_node.location);
            theta = -acosd(dot(old_rocker_pos, new_rocker_pos));
            self.curr_rocker.rotate(theta, new_location);
            
            % Update shock and pushrod 
            self.curr_shock = self.curr_shock.new_outboard_point(new_location);
            self.curr_pushrod.inboard_node.location = self.curr_rocker.control_arm_node.location;
        end
        
        function new_xca = calc_xca_movement(self, xca, ref_point, ref_dist)
            % Calculates the movement of ANY control arm (passive or active) given a reference point, and the distance to that point from a given reference on the control arm
            %
            % Arguments:
            %   xca             (AArm): The control arm to move
            %   ref_point (3x1 Vector): The coord of the reference point
            %   ref_dist       (float): The distance to the reference point
            %
            % Returns:
            %   new_xca         (Aarm): The control arm with updated position
            
            
            if xca.active
                prev_location = xca.pushrod_mount.location;
                [int1, int2] = calc_sphere_circle_int(ref_point, ref_dist,...
                                              xca.pushrod_center, xca.pushrod_radius, xca.pushrod_plane);
                new_location = find_closer_point(prev_location, int1, int2);
                
                new_xca_pos = unit(new_location - xca.pushrod_center);
                old_xca_pos = unit(prev_location - xca.pushrod_center);
                
                assert(xca.pushrod_plane.is_in_plane(new_xca_pos + xca.pushrod_center));
                assert(xca.pushrod_plane.is_in_plane(old_xca_pos + xca.pushrod_center));
            else
                prev_location = xca.tip.location;
                [int1, int2] = calc_sphere_circle_int(ref_point, ref_dist,...
                                              xca.effective_center, xca.effective_radius, xca.action_plane);
                new_location = find_closer_point(prev_location, int1, int2);
                new_xca_pos = unit(new_location - xca.effective_center);
                old_xca_pos = unit(prev_location - xca.effective_center);
            end
            theta = -acosd(dot(old_xca_pos, new_xca_pos));
            new_xca = xca.rotate(theta, new_location);
            
            % Assure that the reference distance is preserved after rotation
            assert(abs(norm(new_location - ref_point) - ref_dist) < 1e-8);
        end
        
        function [cambers, toes, contact_patches] = perform_rack_sweep(self, rack_start_step, rack_step_size, num_steps)
            self.reset_rack();
            self.take_rack_step(rack_start_step);
            b = self.interference_detection(1);
            toes = zeros(1,num_steps);
            cambers = zeros(1,num_steps);
            contact_patches = zeros(3, num_steps);
            [cambers(1), toes(1)] = self.curr_knuckle.calc_camber_and_toe();
            contact_patches(:, 1) = self.curr_knuckle.wheel.contact_patch;
            
            for index = 2:num_steps
                self.take_rack_step(rack_step_size);
                b = self.interference_detection(1);

                [camber, toe] = self.curr_knuckle.calc_camber_and_toe();

                toes(index) = toe;
                cambers(index) = camber;
                contact_patches(:, index) = self.curr_knuckle.wheel.contact_patch;
                if self.plot_on
                    plot_system_3d('g', self.curr_knuckle)
                end
            end
            contact_patches = reshape(contact_patches', [1, num_steps, 3]);
        end
        
        function calc_knuckle_rotation(self)
            % Rotates the knuckle based on current rack location
            %
            % Arguments:
            %   N/A
            %
            % Returns:
            %   N/A
            
            previous_location = self.curr_knuckle.toe_node.location;
            self.curr_knuckle.update_toe_plane();
            
            % Reassign variables to shorten code 
            toe_center = self.curr_knuckle.toe_center;
            toe_radius = self.curr_knuckle.toe_radius;
            toe_plane = self.curr_knuckle.toe_plane;
            
            [p1, p2] = calc_sphere_circle_int(self.curr_rack.endpoint_location, self.toelink_length,...
                                              toe_center, toe_radius, toe_plane);
            
            new_location = find_closer_point(previous_location, p1, p2);
            
            % update toe point of knuckle
            self.curr_knuckle.toe_node.location = new_location;
            self.curr_knuckle.update_action_plane();
            self.curr_knuckle.wheel.update();
        end
        
        function reset_rack(self)
            % Resets the rack to its neutral position and rotates the knuckle accordingly
            % 
            % Arguments:
            %   N/A
            %
            % Returns:
            %   N/A
            
            self.curr_rack.endpoint_location = self.curr_rack.static_endpoint_location;
            self.calc_knuckle_rotation();
        end
        
        function res = interference_detection(self, debug)
            % Checks for interference between suspension components
            % 
            % Arguments:
            %   debug (bool): whether or not to print debug statements
            %
            % Returns:
            %   res   (bool): True if interference is detected, False otherwise
            
            res = true;
            
            % Add some flub room to account for real life errors
            r = self.suspension_member_radius + 0.05;
            previous_interference = self.static_char.interference;
            if ~previous_interference
                self.static_char.interference = true;
            end
            % Check for toe-link aca interference
            if line_line_interference(self.curr_rack.endpoint_location, self.curr_knuckle.toe_node.location, r,...
                                      self.curr_aca.endpoints(1).location, self.curr_aca.tip.location, r)
                if debug
                    disp('toe-link aca interference 1')
                end
                return;
            end
            % Check for toe-link aca interference (second arm)
            if line_line_interference(self.curr_rack.endpoint_location, self.curr_knuckle.toe_node.location, r,...
                                      self.curr_aca.endpoints(2).location, self.curr_aca.tip.location, r)
                if debug
                    disp('toelink aca interference 2')
                end
                return;
            end
            % Check for pushrod toelink interference
            if line_line_interference(self.curr_rack.endpoint_location, self.curr_knuckle.toe_node.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                if debug
                    disp('pushrod toelink interference')
                end
                return;
            end
            
            % Check for pushrod aca interference
            if line_line_interference(self.curr_aca.endpoints(1).location, self.curr_aca.tip.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                if debug
                    disp('pushrod aca interference 1')
                end
                return;
            end
            % Check for pushrod aca interference (second arm)
            if line_line_interference(self.curr_aca.endpoints(2).location, self.curr_aca.tip.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                if debug
                    disp('pushrod aca interference 2')
                end
                return;
            end
            % Check for pushrod pca interference
            if line_line_interference(self.curr_pca.endpoints(1).location, self.curr_pca.tip.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                if debug
                    disp('pushrod pca interference 1')
                end
                return;
            end
            % Check for pushrod pca interference (second arm)
            if line_line_interference(self.curr_pca.endpoints(2).location, self.curr_pca.tip.location, r,...
                                      self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                
                if debug
                    disp('pushrod pca interference 2')
                end
                return;
            end
            
            for index = 1:length(self.curr_knuckle.wheel.radii)
                w = self.curr_knuckle.wheel;
                wr = w.radii(index);
                c = w.center - w.axial_dists(index) * w.axis;
                p = Plane(c, w.axis);
                % Check for toe link interference
                if line_circle_interference(c, wr, p,...
                                            self.curr_rack.endpoint_location, self.curr_knuckle.toe_node.location, r)
                    if debug
                        disp('Wheel toelink interference')
                    end
                    return;
                end
                if line_circle_interference(c, wr, p,...
                                            self.curr_aca.endpoints(1).location, self.curr_aca.tip.location, r)
                    if debug
                        disp('Wheel aca interference 1')
                    end
                    return;
                end
                if line_circle_interference(c, wr, p,...
                                            self.curr_aca.endpoints(2).location, self.curr_aca.tip.location, r)
                    if debug
                        disp('Wheel aca interference 2')
                    end
                    return;
                end
                if line_circle_interference(c, wr, p,...
                                            self.curr_pca.endpoints(1).location, self.curr_pca.tip.location, r)
                    if debug
                        disp('Wheel pca interference 1')
                    end
                    return;
                end
                if line_circle_interference(c, wr, p,...
                                            self.curr_pca.endpoints(2).location, self.curr_pca.tip.location, r)
                    if debug
                        disp('Wheel pca interference 2')
                    end
                    return;
                end
                if line_circle_interference(c, wr, p,...
                                          self.curr_pushrod.inboard_node.location, self.curr_pushrod.outboard_node.location, r)
                    
                    if debug
                        disp('Wheel pushrod interference 1')
                    end
                    return;
                end
            end
            
            res = false;
            
            if ~previous_interference
                self.static_char.interference = false;
            end
        end
        
        function IC = calc_instant_center(self)
            % Calculates instant center
            % 
            % Arguments:
            %   N/A
            %
            % Returns:
            %   N/A
            
            % Project A-arms into front view plane
            [pca_point, pca_line] = self.curr_pca.static_plane.calc_plane_plane_int(self.front_view_plane);
            [aca_point, aca_line] = self.curr_aca.static_plane.calc_plane_plane_int(self.front_view_plane);
            
            % find intersection between projected a-arm lines
            eqns = [pca_line, aca_line, (aca_point - pca_point)];
            sols = rref(eqns);
            n = sols(1, 3);
            IC = pca_point + n * pca_line;
            if abs(dot(unit(IC-aca_point), aca_line)) < 1 - 1e-4
                IC = pca_point - n * pca_line;
                assert(abs(dot(unit(IC-aca_point), aca_line)) < 1- 1e-4);
            end
        end
        
        function RCH = calc_roll_center_height(self, IC)
            % Calculates the roll center height of the car given the instant center
            %
            % Arguments:
            %   IC (3x1 Vector): Coordinate for the instant center
            %
            % Returns:
            %   RCH     (float): Roll center height [in]
            
            self.curr_knuckle.wheel.update();
            cp = self.front_view_plane.project_into_plane(self.curr_knuckle.wheel.contact_patch);
            v = unit(cp - IC);
            n = -IC(1) / v(1);
            RC = IC + n*v;
            RCH = RC(2);
        end
        
        function [angle, mechanical_trail, scrub_radius, spindle_length] = calc_kingpin(self, c, cp)
            % Find which a-arm is the upper vs lower
            if self.curr_pca.tip.location(2) > self.curr_aca.tip.location(2)
                upper_tip = self.curr_pca.tip.location;
                lower_tip = self.curr_aca.tip.location;
                
            else
                lower_tip = self.curr_pca.tip.location;
                upper_tip = self.curr_aca.tip.location;
            end
            angle_sign = sign(lower_tip(1) - upper_tip(1));
            axis = unit(upper_tip - lower_tip);
            front_axis = unit([axis(1); axis(2)]);
            angle = angle_sign * acosd(dot([0;1], front_axis));
            
            m = -lower_tip(2) / axis(2);
            axis_contact = lower_tip + m * axis;
            assert(abs(axis_contact(2)) < 1e-6);
            mechanical_trail = axis_contact(3) - cp(3);
            scrub_radius = axis_contact(1) - cp(1);

            v = [axis_contact, upper_tip];

            lower_tip_2d = [lower_tip(1); lower_tip(2)];
            c_2d = [c(1); c(2)];
            
            a = -dot(front_axis, (c_2d - lower_tip_2d)) / dot(front_axis, front_axis);
            
            spindle_length = norm(c_2d - lower_tip_2d + a * front_axis);
        end
        
        function [SVSA, anti] = calc_SVSA_and_anti(self, c, cp)
            % Calculates the Side-view swing arm length (SVSA) and anti-percentage
            %
            % Arguments:
            %   c  (3x1 Vector): Wheel center coordinate [in]
            %   cp (3x1 Vector): Contact patch coordinate [in]
            %
            % Returns:
            %   SVSA    (float): Length of SVSA [in]
            %   anti    (float): anti-percentage (from 0-1)
            
            [pca_point, pca_line] = self.curr_pca.static_plane.calc_plane_plane_int(self.side_view_plane);
            [aca_point, aca_line] = self.curr_aca.static_plane.calc_plane_plane_int(self.side_view_plane);
            eqns = [pca_line, aca_line, (aca_point - pca_point)];
            sols = rref(eqns);

            % if it is unable to find a solution
            if ~isequal(sols(1:2, 1:2), eye(2))
                SVSA = inf;
                anti = 0;
                return
            end
            n = sols(1, 3);
            IC = pca_point + n * pca_line;
            SVSA = abs(cp(3) - IC(3));
            if strcmp(self.suspension_type, 'front')
                anti = self.front_brake_percentage * (IC(2) / SVSA) * self.wheelbase / self.cgh;
            else
                anti = (abs(IC(2) - c(2)) / SVSA) * self.wheelbase / self.cgh;
            end
        end
        
        function update_all(self)
            % Call all standard update functions for components
            %
            % Arguments:
            %   N/A
            %
            % Returns:
            %   N/A
            
            fprintf('UPDATING SUSPENSION...\n');
            self.curr_rocker.update();
            self.curr_pushrod.update();
            self.curr_aca.update();
            self.curr_pca.update();
            self.curr_knuckle.update();
            self.curr_rack.update();
            self.toelink_length = norm(self.curr_knuckle.toe_node.location - self.curr_rack.endpoint_location);
            self.static_char.interference = 0;
        end
        
        function update_planar_nodes(self, input)
            
            % update action plane
            p1 = self.curr_shock.inboard_node.location;
            p2 = self.curr_pushrod.outboard_node.location;
            p3 = self.curr_rocker.pivot_node.location;
            self.action_plane = Plane(p1, p2, p3);
            if self.action_plane.normal(3) < 0
                self.action_plane.normal = -self.action_plane.normal;
                self.action_plane.update()
            end
            
            shock_angle = input(1);
            pushrod_angle= input(2);
            pushrod_length = input(3);
            shock_point = calc_planar_link_from_angle(self.action_plane,...
                                  self.curr_shock.inboard_node.location,...
                                  self.curr_shock.static_length,...
                                  self.curr_rocker.pivot_node.location,... 
                                  shock_angle);
                              
            pushrod_point = calc_planar_link_from_angle(self.action_plane,...
                                  self.curr_pushrod.outboard_node.location,...
                                  pushrod_length,...
                                  self.curr_rocker.pivot_node.location,... 
                                  pushrod_angle);
                              
            self.curr_shock.outboard_node.location = shock_point;
            self.curr_pushrod.inboard_node.location = pushrod_point;
            self.curr_shock.curr_length = self.curr_shock.calc_length();
        end
        
        function output = get_planar_data(self)
            % Function finds data from planar nodes (ie: rocker points
            % other than the pivot)
            
            % output(1) is the angle between the shock and the line
            %   between the shock and the rocker pivot
            % output(2) is the angle between the pushrod and the line
            %   between the pushrod and the rocker pivot
            % output(3) is the current length of the pushrod
            
            % Create shorthands to shorten code length
            pivot = self.curr_rocker.pivot_node.location;
            shock_out = self.curr_rocker.shock_node.location;
            shock_in = self.curr_shock.inboard_node.location;
            pushrod_out = self.curr_pushrod.outboard_node.location;
            pushrod_in = self.curr_pushrod.inboard_node.location;
            
            v1 = unit(shock_out-shock_in);
            v2 = unit(pivot - shock_in);
            shock_angle = acosd(dot(v1, v2));
            cross_direction = cross(v2, v1);
            shock_angle = shock_angle * sign(dot(cross_direction, self.action_plane.normal));
            
            v1 = unit(pushrod_in - pushrod_out);
            v2 = unit(pivot - pushrod_out);
            pushrod_angle = acosd(dot(v1, v2));
            cross_direction = cross(v2, v1);
            pushrod_angle = pushrod_angle * sign(dot(cross_direction, self.action_plane.normal));
            
            output = [shock_angle; pushrod_angle; self.curr_pushrod.length];
        end
        
        function calc_static_char(self)
            % Calculates the static characteristics of the geometry.
            % Updates self.static_char
            %
            % Arguments: 
            %   N/A
            %
            % Returns:
            %   N/A
            
            c = self.curr_knuckle.wheel.center;
            cp = self.curr_knuckle.wheel.contact_patch;
            
            IC = self.calc_instant_center();
            self.static_char.RCH = self.calc_roll_center_height(IC);
            self.static_char.FVSA = abs(IC(1)-cp(1));
            
           
            [angle, mechanical_trail, scrub_radius, spindle_length] = self.calc_kingpin(c, cp);
            self.static_char.kingpin_angle = angle;
            self.static_char.mechanical_trail = mechanical_trail;
            self.static_char.scrub_radius = scrub_radius;
            self.static_char.spindle_length = spindle_length;
            [SVSA, anti] = self.calc_SVSA_and_anti(c, cp);
            self.static_char.SVSA = SVSA;
            self.static_char.anti_percentage = anti;
        end
    end
end