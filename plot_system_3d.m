function plot_system_3d(varargin)
    assert(nargin > 1)
    color = varargin{1};
    axis equal
    xlabel('x (in)')
    ylabel('y (in)')
    zlabel('z (in)')
    view(3)
    camup('manual') 
    camup([0 1 0])
    cameratoolbar('Show')
    for plot_element = 2:nargin
        curr_element = varargin{plot_element};
        if isa(curr_element, 'Line')
            v = [curr_element.inboard_point, curr_element.outboard_point];
            plot3(v(1, :), v(2, :), v(3, :), color)
        end
        
        if isa(curr_element, 'AArm')
            endpoints = curr_element.get_endpoint_locations;
            tip = curr_element.tip.location;
            v1 = [endpoints(:, 1), tip];
            v2 = endpoints(:, 2);
            v = [v1, v2];
            plot3(v(1, :), v(2, :), v(3, :), color)
            scatter3(curr_element.pivots(1, :), curr_element.pivots(2, :), curr_element.pivots(3, :), 20, 'filled', color);
            
            pivot_axis = curr_element.pivots;
            plot3(pivot_axis(1, :), pivot_axis(2, :), pivot_axis(3, :), 'Color', color, 'LineStyle', '--');
            center = curr_element.effective_center;
            scatter3(center(1), center(2), center(3), 20, 'k');
        end
        
        if isa(curr_element, 'Node')
            p = curr_element.location;
            scatter3(p(1), p(2), p(3), 20, 'filled', color)
        end
        
        if isa(curr_element, 'Rocker')
            v = [curr_element.ordered_points, curr_element.pivot_point];
            plot3(v(1, :), v(2, :), v(3, :), color)
            scatter3(v(1, :), v(2, :), v(3, :), 20, 'filled', color);
        end
        
        if isa(curr_element, 'Shock')
            v = [curr_element.outboard_point, curr_element.inboard_node.location];
            plot3(v(1, :), v(2, :), v(3, :), 'Color', color, 'LineStyle', '-.')
            scatter3(v(1, :), v(2, :), v(3, :), 20, 'filled', color);
        end
        
        if isa(curr_element, 'Knuckle')
            v = curr_element.lca_point.location;
            v2 = curr_element.toe_point;
            scatter3(v(1), v(2), v(3), 'r*');
            scatter3(v2(1), v2(2), v2(3), color, '*');
            
            n_pts = 15;
            C = curr_element.wheel.center;

            rs = curr_element.wheel.radii;
            ds = curr_element.wheel.axial_dists;
            rot_axis = curr_element.wheel.axis;
            p = curr_element.wheel.plane;
            A = unit(C - p.project_into_plane(C + [0; 1; 0]));
            B = unit(cross(A, rot_axis));
            thetas = linspace(0, 360, n_pts);
            ap = curr_element.wheel.axis_point;
            scatter3(ap(1), ap(2), ap(3), 20, 'm*')
            for index = 1:length(ds)
                d = ds(index);
                r = rs(index);
                a = rot_axis * d + A;
                b = rot_axis * d + B;
                c = rot_axis * d + C;
                pts = c + (r * a * sind(thetas)) + (r * b * cosd(thetas));
                plot3(pts(1, :), pts(2, :), pts(3, :), color)
                quiver3(c(1), c(2), c(3), a(1)*r, a(2)*r, a(3)*r)
                quiver3(c(1), c(2), c(3), b(1)*r, b(2)*r, b(3)*r)
                quiver3(c(1), c(2), c(3), rot_axis(1)*10, rot_axis(2)*10, rot_axis(3)*10)
                scatter3(c(1), c(2), c(3), 20, 'filled', color);
            end
            
            
        end
        
        if isa(curr_element, 'Rack')
            p = curr_element.location_node.location;
            v1 = curr_element.endpoint_location;
            v2 = p - (curr_element.static_length / 2) * [1;0;0];
            v = [v1, v2];
            
            scatter3(p(1), p(2), p(3), 20, color)
            plot3(v(1, :), v(2, :), v(3, :), 'Color', color)
        end
            
    end
end