addpath Classes

clf
d = [0.125; 0.125; 0.125]*4;

n_pushrod_out = Node([21.71; 6.57; -30.58], 0*d);
% n_pushrod_out.shuffle();

lower_tip = [23.65; 5.48; -30.40];

n1 = Node([10.47; 6.54-1.5; -24.01-2], d);
n2 = Node([10.47; 5.95-1.5; -34.49], d*2);
n3 = Node(lower_tip, d*2);
% n1.shuffle();
% n2.shuffle();
% n3.shuffle();

lower_wishbone = AArm(n3, n2, n1, n_pushrod_out);



upper_tip = [23.34; 12.48; -30.60];
n11 = Node([11.43; 11.09+1; -22.5-3], d);
n22 = Node([11.42; 11.07+1; -34.5], d*2);
n33 = Node(upper_tip, d*2);
% n11.shuffle();
% n22.shuffle();
% n33.shuffle();
upper_wishbone = AArm(n33, n22, n11);

wheel = Wheel(-1, -1, [25; 8.98; -30.5], [9, 4.75, 4.75, 5.06], [0, 0, 2.92, 3.3]);

n_outboard_toe = Node([23.32; 12.48; -29.10], d);
% n_outboard_toe.shuffle();

knuckle = Knuckle(upper_wishbone.tip, lower_wishbone.tip, n_outboard_toe, wheel);

n_rocker_pivot = Node([10.32; 13.58; -32.97], d*0);
% n_rocker_pivot.shuffle();

p_shock_in = [7.74; 6.29; -33.24];
n_shock = Node(p_shock_in, d*0);
% n_shock.shuffle();

action_plane = Plane(n_pushrod_out.location, n_shock.location, n_rocker_pivot.location);

p_rocker_shock = [7.67; 13.06; -33.46];
n_rocker_shock = Node(p_rocker_shock, d*0);
% n_rocker_shock.shuffle();
n_rocker_shock.location = action_plane.project_into_plane(p_rocker_shock);

n_pushrod_in = Node([11.06; 14.58; -32.86], d*0);
% n_pushrod_in.shuffle();
n_pushrod_in.location = action_plane.project_into_plane(n_pushrod_in.location);
pushrod = Line(n_pushrod_in, n_pushrod_out);

rocker = Rocker(n_rocker_pivot, n_rocker_shock, n_pushrod_in);

rack_node = Node([0;11.09;-19.82], d);
% rack_node.shuffle();

rack = Rack(rack_node, 0, 11.42*2);


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
    [static_char, dyn_char] = ag.perform_sweep(6, 0);
end
toc

desired_static_char = struct('RCH', 4.5,...
                             'spindle_length', 1,...
                             'kingpin_angle', 1.5,...
                             'scrub_radius', 1.1,...
                             'anti_percentage', 0.25,...
                             'FVSA', 42.5,...
                             'SVSA', 134.2,...
                             'mechanical_trail', 0.25,...
                             'interference', 0);

static_char_weights = struct('RCH', 80,...
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

