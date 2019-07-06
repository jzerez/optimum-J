addpath Classes

clf
bounding = [-1, -1, -1, -1,  1,  1,  1,  1;...
            -1, -1,  1,  1, -1, -1,  1,  1;...
            -1,  1, -1,  1, -1,  1, -1,  1;];
p4 = [23.49; 11.46; 30.35];
bounding = bounding * 200;
r = Region(bounding);
bounding2 = p4 + (bounding * 1.2);
r2 = Region(bounding2);


n_pushrod_in = Node([9.52; 4.42; 30.69], r);
n_pushrod_out = Node(p4, r2);
pushrod = Line(n_pushrod_in, n_pushrod_out);

p1 = [8.4; 4.5; 25.6];
p2 = [7.8; 4.5; 36.9];
p3 = [24.2; 5.73; 30.7];

n1 = Node(p1, r);
n2 = Node(p2, r);
n3 = Node(p3, r);

lower_wishbone = AArm(n3, n2, n1);

p11 = [9.3; 12.7; 36.5];
p22 = [9.2; 12.5; 25.7];


p33 = [24.0; 12.2; 30.3];
n11 = Node(p11, r);
n22 = Node(p22, r);
n33 = Node(p33, r);
upper_wishbone = AArm(n33, n22, n11, n_pushrod_out);

wheel = Wheel(-1, -1, [25; 9.88; 30], [], []);

p_inboard_toe = [8.34; 4.5; 25.67];
p_outboard_toe = [25.07; 8.98; 26.52];
knuckle = Knuckle(lower_wishbone.tip, upper_wishbone.tip, p_outboard_toe, false, wheel);

p_rocker_pivot = [8.62; 5.37; 30.75];
action_plane = Plane(n_pushrod_in.location, n_pushrod_out.location, p_rocker_pivot);

p_rocker_shock = [8.39; 6.65; 30.79];
p_rocker_arb = [9.29; 5.7; 30.73];
p_rocker_shock = action_plane.project_into_plane(p_rocker_shock);
p_rocker_arb = action_plane.project_into_plane(p_rocker_arb);


rocker = Rocker(p_rocker_pivot, p_rocker_shock, n_pushrod_in.location, p_rocker_arb);

rack_node = Node([0;4.5;25.67], r);

rack = Rack(rack_node, 2, 12.1);
p_shock_in = action_plane.project_into_plane([1.97; 4.5; 30.79]);
n_shock = Node(p_shock_in, r);
shock = Shock(n_shock, p_rocker_shock, action_plane);

ag = ActionGroup(rocker, shock, pushrod, upper_wishbone, lower_wishbone, knuckle, rack);

hold on

plot_system_3d('y', lower_wishbone, upper_wishbone, knuckle)
plot_system_3d('y', n3, n33)
plot_system_3d('y', rocker)
plot_system_3d('k', rack)

tic
for iter = 1:1
    hold on
    ag.perform_sweep(7, 1);
end
toc
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
