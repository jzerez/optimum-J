addpath Classes

clf

p4 = [23.2; 6.73; 30.7];

d = [1.5; 1.5; 1.5];

n_pushrod_in = Node([11.1; 15.0; 27.6], d);
n_pushrod_out = Node(p4, d);
pushrod = Line(n_pushrod_in, n_pushrod_out);

p1 = [8.4; 4.5; 25.6];
p2 = [7.8; 4.5; 36.9];
p3 = [24.2; 5.73; 30.7];

n1 = Node(p1, d);
n2 = Node(p2, d);
n3 = Node(p3, d);

lower_wishbone = AArm(n3, n2, n1, n_pushrod_out);

p11 = [9.3; 12.7; 36.5];
p22 = [9.2; 12.5; 25.7];


p33 = [24.0; 12.2; 30.3];
n11 = Node(p11, d);
n22 = Node(p22, d);
n33 = Node(p33, d);
upper_wishbone = AArm(n33, n22, n11);

wheel = Wheel(-1, -1, [25; 9.88; 30], [9], [0]);

n_outboard_toe = Node([24.39; 6.63; 32.98], d);

knuckle = Knuckle(upper_wishbone.tip, lower_wishbone.tip, n_outboard_toe, wheel);

n_rocker_pivot = Node([10.2; 13.6; 27.2], d);
action_plane = Plane(n_pushrod_in.location, n_pushrod_out.location, n_rocker_pivot.location);

p_rocker_shock = [12.4; 15.4; 28.1];
n_rocker_shock = Node(action_plane.project_into_plane(p_rocker_shock), d);


rocker = Rocker(n_rocker_pivot, n_rocker_shock, n_pushrod_in);

rack_node = Node([0;6.75;30.2], d);

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
for iter = 1:0
    hold on
    [static_char, ~] = ag.perform_sweep(6, 1);
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
static_char_weights = struct('RCH', 10,...
                             'spindle_length', 2,...
                             'kingpin_angle', 2,...
                             'scrub_radius', 3,...
                             'anti_percentage', 6,...
                             'FVSA', 7,...
                             'SVSA', 7,...
                             'mechanical_trail', 5,...
                             'interference', 50);
% fitness = calc_fitness(static_char, desired_static_char, static_char_weights)
o = Optimizer(ag, 10, desired_static_char, static_char_weights);

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

