addpath Classes
clf
bounding = [-1, -1, -1, -1,  1,  1,  1,  1;...
            -1, -1,  1,  1, -1, -1,  1,  1;...
            -1,  1, -1,  1, -1,  1, -1,  1;];
bounding2 = bounding + [1;2;2];
bounding = bounding * 200;
r =  Region(bounding, bounding2);
r.is_within_region([1.4;2;2])

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
upper_wishbone = AArm(n33, n22, n11);

p_inboard_toe = [6.1; 6.75; 30.2];
p_outboard_toe = [24.39; 6.63; 32.98];
knuckle = Knuckle(upper_wishbone.tip, lower_wishbone.tip, p_outboard_toe, -0.75, -1, true);

p_pushrod_in = [11.1; 15.0; 27.6];
p_pushrod_out = p3;
pushrod = Line(p_pushrod_in, p_pushrod_out);

p_rocker_pivot = [10.2; 13.6; 27.2];
action_plane = Plane(p_pushrod_in, p_pushrod_out, p_rocker_pivot);

p_rocker_shock = [12.4; 15.4; 28.1];
p_rocker_arb = [11.1; 13.4; 27.5];
p_rocker_shock = action_plane.project_into_plane(p_rocker_shock);
p_rocker_arb = action_plane.project_into_plane(p_rocker_arb);


rocker = Rocker(p_rocker_pivot, p_rocker_shock, p_pushrod_in, p_rocker_arb);

rack_node = Node([0;6.75;30.2], r);

rack = Rack(rack_node, 2, 12.1);
p_shock_in = action_plane.project_into_plane([8.3; 20.7; 27.2]);
n_shock = Node(p_shock_in, r);
shock = Shock(n_shock, p_rocker_shock, action_plane);

testag = ActionGroup(rocker, shock, pushrod, lower_wishbone, upper_wishbone, knuckle, rack);

hold on

plot_system_3d('y', lower_wishbone, upper_wishbone, knuckle)
plot_system_3d('y', n3, n33)
plot_system_3d('y', rocker, pushrod, knuckle)
plot_system_3d('k', rack)
 
tic
for iter = 1:1
    hold on
    ag.perform_sweep(6, 1);
end
toc
% 50000 simulations = 72 seconds as of 3/9/19



