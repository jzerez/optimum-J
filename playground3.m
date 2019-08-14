addpath Classes

clf
d = [1.5; 1.5; 1.5];
p4 = [23.2; 7.73; 30.7];

n_pushrod_in = Node([11.1; 15.0; 27.6], d);
n_pushrod_out = Node(p4, d);
pushrod = Line(n_pushrod_in, n_pushrod_out);


lower_tip = [21.2; 7.5; 30.7];

n1 = Node(NaN, [7, 10; 2, 8; 23, 29]');
n2 = Node(NaN, [7, 10; 2, 8; 36, 38]');
n3 = Node(lower_tip, [3; 0.5; 1.5]);

lower_wishbone = AArm(n3, n2, n1, n_pushrod_out);



upper_tip = [21.0; 12.2; 30.3];
n11 = Node(NaN, [7, 10; 12.4, 14.6; 23, 27]');
n22 = Node(NaN, [7, 11; 10, 15; 33, 39]');
n33 = Node(upper_tip, [3; 0.5; 1.5]);
upper_wishbone = AArm(n33, n22, n11);

wheel = Wheel(-1, -1, [25; 9.88; 30], [9, 4.75, 4.75, 5.1], [0, 0, 3.3, 3.88]);

n_outboard_toe = Node([21.39; 8.63; 32.98], d*6);

knuckle = Knuckle(upper_wishbone.tip, lower_wishbone.tip, n_outboard_toe, wheel);

n_rocker_pivot = Node(NaN, [7, 10; 12.4, 14.6; 23, 28]');
action_plane = Plane(n_pushrod_in.location, n_pushrod_out.location, n_rocker_pivot.location);

p_rocker_shock = [12.4; 15.4; 28.1];
n_rocker_shock = Node(action_plane.project_into_plane(p_rocker_shock), d);


rocker = Rocker(n_rocker_pivot, n_rocker_shock, n_pushrod_in);

rack_node = Node([0;6.75;30.2], d*3);

rack = Rack(rack_node, 2, 12.1);
p_shock_in = action_plane.project_into_plane([8.3; 20.7; 27.2]);
n_shock = Node(p_shock_in, d);
shock = Shock(n_shock, n_rocker_shock, action_plane);

ag = ActionGroup(rocker, shock, pushrod, lower_wishbone, upper_wishbone, knuckle, rack, 'front');

hold on

plot_system_3d('y', lower_wishbone, upper_wishbone, knuckle)
plot_system_3d('y', n3, n33)
plot_system_3d('y', rocker, pushrod, knuckle)
plot_system_3d('k', rack)

tic
for iter = 1:1
    hold on
    [static_char, dyn_char] = ag.perform_sweep(6, 1);
end
toc

desired_static_char = struct('RCH', 4.5,...
                             'spindle_length', 1,...
                             'kingpin_angle', 1.5,...
                             'scrub_radius', 1.5,...
                             'anti_percentage', 0.25,...
                             'FVSA', 69,...
                             'SVSA', 110,...
                             'mechanical_trail', 1,...
                             'interference', 0);

static_char_weights = struct('RCH', 45,...
                             'spindle_length', 2,...
                             'kingpin_angle', 2,...
                             'scrub_radius', 3,...
                             'anti_percentage', 6,...
                             'FVSA', 5,...
                             'SVSA', 5,...
                             'mechanical_trail', 5,...
                             'interference', 100);
                         
desired_dyn_char = struct('bump_steer', 0,...
                          'min_wheel_travel', 1.98,...
                          'wheel_travel', 2.1,...
                          'scrub', 0);
                      
dyn_char_weights = struct('bump_steer', 5,...
                          'min_wheel_travel', 100,...
                          'wheel_travel', 1,...
                          'scrub', 3);
                         

% fitness = calc_fitness(static_char, desired_static_char, static_char_weights)
o = Optimizer(ag, 10, desired_static_char, static_char_weights, desired_dyn_char, dyn_char_weights);

% 50000 simulations = 72 seconds as of 3/9/19 (Shock Sweep only)
% 5000 Simulations = 120 seconds as of 3/19/19 (Shock and Rack Sweep)
% 5000 Simulations = 212 seconds as of 3/23/19 (shock, and rack sweep, 6x6, line
% interference detection)
% 5000 Simulations = 367 seconds as of 3/31/19 (shock, rack sweep, 6x6,
% line and circle interference detection, IC, RC, wheel travel, some object optimization,
% printing)
% 5000 Simulations = 261 seconds as of 3/31/19 (shock, rack sweep, 6x6,
% line and circle interference detection, IC, RC, wheel travel, some object optimization,
% less printing)
% 5000 Simulations = 234 seconds as of 4/16/19 (shock, rack sweep, 6x6,
% line and circle interference detection, IC, RC, wheel travel, better object
% optimization)

