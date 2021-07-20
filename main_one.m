%%% This is one of two main scripts used to run suspension optimization %%%
% 
% It will initialize a suspension and try to optimize ONCE for the given
% initial conditions. 
%
% All component positions are for the car's STATIC EQUILIBRIUM
% (ie: Car on the ground and unmoving)
%
% All units are inches or degrees
%
% Coordinate system:
% X: Lateral (left positive)
% Y: Vertical (up positive)
% Z: Longitudinal (front positive)
% Origin: located along the car center line, on the ground, and at half the
% wheelbase

% add directories to the path so we can utilize files located there
addpath Classes
addpath Functions

clf

%% Section 1: Initializing the Suspension
%%% The size of each bounding region for each node in inches
d = [0.125; 0.125; 0.125]*6;

%%% Node for the outboard pushrod
n_pushrod_out = Node([22.79; 6.5; 30.72], d);

%%% Location for the outboard lower a-arm point
lower_tip = [23.7; 5.23; 30.72];

%%% Nodes for the lower A-arm
% n1: lower rear node
% n2: lower front node
% n3: lower a-arm tip
n1 = Node([9.01+1; 5.848; 26.63], d*1.5);
n2 = Node([9.01; 5.576; 36.62], d*1.5);
n3 = Node(lower_tip, d);

%%% Creates the lower a-arm object
lower_wishbone = AArm(n3, n2, n1, n_pushrod_out);

%%% Location for the outboard upper a-arm point
upper_tip = [24.33; 12.87; 29.82];

%%% Nodes for the lower A-arm
% n11: upper rear node
% n22: upper front node
% n33: upper a-arm tip
n11 = Node([9.48; 11.07; 26.49], d*1.5);
n22 = Node([9.48; 11.05; 36.49], d*1.5);
n33 = Node(upper_tip, d);

%%% Create upper a-arm
upper_wishbone = AArm(n33, n22, n11);

%%% Creates the wheel object:
% The radii and axial dists are measured from our physical wheels
% The other arguments are design variables (static camber, toe, wheel
% center location)
wheel = Wheel(-1, -1, [25; 8.98; 30.5], [9, 4.75, 4.75, 5.06], [0, 0, 2.92, 3.3]);

%%% Node for the outboard toe link
n_outboard_toe = Node([24.38; 6; 32.98], d);

%%% Creates the Knuckle Object
knuckle = Knuckle(lower_wishbone.tip, upper_wishbone.tip, n_outboard_toe, wheel);

%%% Creates the Rocker pivot point node
n_rocker_pivot = Node([7.58; 4.33; 30.25], d);

%%% The inboard location for the shock mount point
p_shock_in = [0.88; 3.45; 30.25];

%%% The node for the inboard shock mount point
n_shock = Node(p_shock_in, d);

%%% The plane that the rocker acts in
action_plane = Plane(n_pushrod_out.location, n_shock.location, n_rocker_pivot.location);

%%% Location of the shock mount point on the rocker
p_rocker_shock = [7.66; 2.45; 30.25];
n_rocker_shock = Node(p_rocker_shock, d);

%%% Project the shock mount point on the rocker into the rocker's plane 
% (This eliminates any bending loads in the rocker in real life)
n_rocker_shock.location = action_plane.project_into_plane(p_rocker_shock);

%%% Node for the inboard pushrod point (ie: on the rocker)
n_pushrod_in = Node([7.45; 5.3; 30.25], d);

%%% Project the pushrod mount point on the rocker into the rocker's plane 
% (This eliminates any bending loads in the rocker in real life)
n_pushrod_in.location = action_plane.project_into_plane(n_pushrod_in.location);

%%% Creates pushrod object
pushrod = Line(n_pushrod_in, n_pushrod_out);

%%% Creates rocker object
rocker = Rocker(n_rocker_pivot, n_rocker_shock, n_pushrod_in);

%%% Node for the center point of the steering rack
rack_node = Node([0;6.75;30.17], d*2);

%%% Creates the rack object
rack = Rack(rack_node, 2, 12.1);

%%% Creates the shock object
shock = Shock(n_shock, n_rocker_shock, action_plane);

%%% Creates the suspension object
% (ok, yes. ActionGroup is a dumb name for it. Sue me.)
ag = ActionGroup(rocker, shock, pushrod, upper_wishbone, lower_wishbone, knuckle, rack, 'front');

%%% Plot the initial suspension
hold on
plot_system_3d('y', lower_wishbone, upper_wishbone, knuckle)
plot_system_3d('y', n3, n33)
plot_system_3d('y', rocker, pushrod, knuckle)
plot_system_3d('k', rack)

%%% Sweep the suspension through its range of motion (and time it)
% This ensures the suspension starts in a "working" state
%
% This also calculates the initial static and dynamic characteristics for
% the suspension
tic
[static_char, dyn_char] = ag.perform_sweep(6, 1);
toc

%% Section 2: Define Design Goals
% As of 06/20/2021, these values are loosely "correct" for the front
% suspension


%%% Desired STATIC chracteristics
desired_static_char = struct('RCH', 4,...
                             'spindle_length', 1,...
                             'kingpin_angle', 1.5,...
                             'scrub_radius', 1.5,...
                             'anti_percentage', 0.25,...
                             'FVSA', 69,...
                             'SVSA', 110,...
                             'mechanical_trail', 1,...
                             'interference', 0);

%%% Weighs for STATIC characteristics (how much we care about each one)                         
static_char_weights = struct('RCH', 80,...
                             'spindle_length', 2,...
                             'kingpin_angle', 2,...
                             'scrub_radius', 3,...
                             'anti_percentage', 6,...
                             'FVSA', 5,...
                             'SVSA', 5,...
                             'mechanical_trail', 5,...
                             'interference', 100);

%%% Desired DYNAMIC characteristics
desired_dyn_char = struct('bump_steer', 0,...
                          'min_wheel_travel', 1.98,...
                          'wheel_travel', 2.1,...
                          'scrub', 0);
                      
%%% Weighs for DYNAMIC characteristics (how much we care about each one)                        
dyn_char_weights = struct('bump_steer', 5,...
                          'min_wheel_travel', 100,...
                          'wheel_travel', 1,...
                          'scrub', 3);


%%% To check the intial fitness of the suspension design, uncomment below
% fitness = calc_fitness(static_char, desired_static_char, static_char_weights)

%% Section 3: Running the optimizer!
o = Optimizer(ag, 10, desired_static_char, static_char_weights, desired_dyn_char, dyn_char_weights);
