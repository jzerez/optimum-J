function out = print_solidworks_vector(ag)
    ca1 = ag.curr_pca;
    ca2 = ag.curr_aca;
    
    out = zeros([24,1]);
    
    if ca1.tip.location(2) > ca2.tip.location(2)
        upper_a_arm = ca1;
        lower_a_arm = ca2;
    else
        upper_a_arm = ca2;
        lower_a_arm = ca1;
    end
    
    if lower_a_arm.endpoints(1).location(3) > lower_a_arm.endpoints(2).location(3)
        flf = lower_a_arm.endpoints(1).location;
        flr = lower_a_arm.endpoints(2).location;
    else
        flr = lower_a_arm.endpoints(1).location;
        flf = lower_a_arm.endpoints(2).location;
    end
    
    if upper_a_arm.endpoints(1).location(3) > upper_a_arm.endpoints(2).location(3)
        fuf = upper_a_arm.endpoints(1).location;
        fur = upper_a_arm.endpoints(2).location;
    else
        fur = upper_a_arm.endpoints(1).location;
        fuf = upper_a_arm.endpoints(2).location;
    end
    
    fut = upper_a_arm.tip.location;
    flt = lower_a_arm.tip.location;
    
    rocker = ag.curr_rocker.pivot_node.location;
    shock_in = ag.curr_shock.inboard_node.location;
    
    out = [flr; flf; flt; fur; fuf; fut; rocker; shock_in];
    out = out';
end