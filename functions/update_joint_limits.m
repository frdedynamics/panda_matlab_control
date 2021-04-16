function [bounds] = update_joint_limits(hw_bounds,max_change,current_q)
%UPDATE_JOINT_LIMITS Summary of this function goes here
%   Detailed explanation goes here

tmp_bounds = hw_bounds;

for joint=1:7
    
    if (current_q(joint) - max_change(joint)) > hw_bounds(joint,1) % lower bound
        tmp_bounds(joint,1) = current_q(joint) - max_change(joint);
    end
    if (current_q(joint) + max_change(joint)) < hw_bounds(joint,2) % upper bound
        tmp_bounds(joint,2) = current_q(joint) + max_change(joint);
    end
        
end

bounds = tmp_bounds;

end

