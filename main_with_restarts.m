%%% This is one of two main scripts used to run suspension optimization %%%
% 
% It will initialize a suspension and try to optimize ONCE for the given
% initial conditions. It will then RESEED/RANDOMIZE the initial conditions
% and run the optimization again. This is useful because this optimization
% problem is very sensitive to initial conditions. 
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

clear all
%% Section 1: Define Design Goals
% As of 06/20/2021, these values are loosely "correct" for the rear suspension

%%% Desired STATIC chracteristics
desired_static_char = struct('RCH', 4.5,...
                             'spindle_length', 0,...
                             'kingpin_angle', 1.5,...
                             'scrub_radius', 0,...
                             'anti_percentage', 0.25,...
                             'FVSA', 42.5,...
                             'SVSA', 134.2,...
                             'mechanical_trail', 0.25,...
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
                          'scrub', 0,...
                          'max_steer_angle', 1);

%%% Weighs for DYNAMIC characteristics
dyn_char_weights = struct('bump_steer', 5,...
                          'min_wheel_travel', 100,...
                          'wheel_travel', 1,...
                          'scrub', 1,...
                          'max_steer_angle', 0);

%% Section 2: Initialize and Optimize

start_time = tic;
num = 0;

%%% Name of the output directory folder
path = "./Outputs/";

%%% The size of each bounding region for each node in inches
d = [0.125; 0.125; 0.125]*8;
clf

%%% Number of re-seedings and optimizations to perform
for i = 1:5
    %%% Node for the outboard pushrod. 
    n_pushrod_out = Node([21.71; 6.57; -30.58], 0*d);
    % Randomize location (within allowable region) 
    n_pushrod_out.shuffle();
    
    %%% Location for the outboard lower a-arm point
    lower_tip = [24.52; 6; -30.5];
    
    %%% Nodes for the lower A-arm
    % n1: lower rear node
    % n2: lower front node
    % n3: lower a-arm tip
    n1 = Node([11.26; 6.54-1; -26.63], d);
    n2 = Node([11.26; 5.95-1; -35.99], d);
    n3 = Node(lower_tip, [1; 0.4; 0.4]);
    % Randomize location (within allowable region)
    n1.shuffle();
    n2.shuffle();
    n3.shuffle();
    
    %%% Creates the lower a-arm object
    lower_wishbone = AArm(n3, n2, n1, n_pushrod_out);


    %%% Location for the outboard upper a-arm point
    upper_tip = [24.34; 11.75; -31];
    
    %%% Nodes for the lower A-arm
    % n11: upper rear node
    % n22: upper front node
    % n33: upper a-arm tip
    n11 = Node([11.43; 11.11+1; -26.29], d);
    n22 = Node([11.42; 11.04+1; -35.99], d);
    n33 = Node(upper_tip, [1; 0.3; 0.4]);
    % Randomize location (within allowable region)
    n11.shuffle();
    n22.shuffle();
    n33.shuffle();
    
    %%% Create upper a-arm
    upper_wishbone = AArm(n33, n22, n11);
    
    %%% Creates the wheel object:
    % The radii and axial dists are measured from our physical wheels
    % The other arguments are design variables (static camber, toe, wheel
    % center location)
    wheel = Wheel(-1, -1, [25; 8.98; -30.5], [9, 4.75, 4.75, 5.06], [0, 0, 2.92, 3.3]);
    
    %%% Node for the outboard toe link
    n_outboard_toe = Node([23.32; 12.48; -29.10], d/3);
    % Randomize location (within allowable region)
    n_outboard_toe.shuffle();
    
    %%% Creates the Knuckle Object
    knuckle = Knuckle(upper_wishbone.tip, lower_wishbone.tip, n_outboard_toe, wheel);
    
    %%% Creates the Rocker pivot point node
    n_rocker_pivot = Node([10.32; 13.58; -32.97], d*0);
    % Randomize location (within allowable region)
    n_rocker_pivot.shuffle();
    
    %%% The inboard location for the shock mount point
    p_shock_in = [7.74; 6.29; -33.24];
    n_shock = Node(p_shock_in, d*0);
    % Randomize location (within allowable region)
    n_shock.shuffle();
    
    %%% The plane that the rocker acts in
    action_plane = Plane(n_pushrod_out.location, n_shock.location, n_rocker_pivot.location);
    
    %%% Location of the shock mount point on the rocker
    p_rocker_shock = [7.67; 13.06; -33.46];
    n_rocker_shock = Node(p_rocker_shock, d*0);
    % Randomize location (within allowable region)
    n_rocker_shock.shuffle();
    
    %%% Project the shock mount point on the rocker into the rocker's plane 
    % (This eliminates any bending loads in the rocker in real life)
    n_rocker_shock.location = action_plane.project_into_plane(p_rocker_shock);
    
    %%% Node for the inboard pushrod point (ie: on the rocker)
    n_pushrod_in = Node([11.06; 14.58; -32.86], d*0);
    % Randomize location (within allowable region)
    n_pushrod_in.shuffle();
    
    %%% Project the pushrod mount point on the rocker into the rocker's plane 
    % (This eliminates any bending loads in the rocker in real life)
    n_pushrod_in.location = action_plane.project_into_plane(n_pushrod_in.location);
    
    %%% Creates pushrod object
    pushrod = Line(n_pushrod_in, n_pushrod_out);
    
    %%% Creates rocker object
    rocker = Rocker(n_rocker_pivot, n_rocker_shock, n_pushrod_in);
    
    %%% Node for the center point of the steering rack
    rack_node = Node([0;11.09;-19.82-3], d);
    % Randomize location (within allowable region)
    rack_node.shuffle();
    
    %%% Creates the rack object
    rack = Rack(rack_node, 0, 11.42*2);

    %%% Creates the shock object
    shock = Shock(n_shock, n_rocker_shock, action_plane);
    
    %%% Creates the suspension object
    % (ok, yes. ActionGroup is a dumb name for it. Sue me.)
    ag = ActionGroup(rocker, shock, pushrod, lower_wishbone, upper_wishbone, knuckle, rack, 'rear');
    
    %%% Plot the initial suspension
    hold on
    plot_system_3d('y', lower_wishbone, upper_wishbone, knuckle)
    plot_system_3d('y', n3, n33)
    plot_system_3d('y', rocker, pushrod, knuckle)
    plot_system_3d('k', rack)

    %%% Sweep the suspension through its range of motion (and time it)
    % This ensures the suspension starts in a "working" state
    tic  
    [static_char, dyn_char] = ag.perform_sweep(6, 0);
    toc
    
    %%% Optimize! (And ignore any errors that may come up lol)
    try
        o = Optimizer(ag, 10, desired_static_char, static_char_weights, desired_dyn_char, dyn_char_weights);
    catch
        continue;
    end
    
    %%% Only accept solutions that meet a fitness threshold
    % Save "optimal" suspension designs as .mat files into the output
    % directory
    if min(o.fitnesses) < 1600
        filename = path + "rear-" + num2str(round(min(o.fitnesses))) + "-";
        tag = 0;
        while isfile(filename + num2str(tag) + ".mat")
            tag = tag + 1;
        end
        filename = filename + num2str(tag);
        save(filename)
        num = num+1;
    end
end
disp(toc(start_time))
disp(num)
