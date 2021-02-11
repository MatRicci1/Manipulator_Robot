function G = ComputeGravTorque(robot,q)
%ComputeGravTorque
% Computes the resistives torques in the joints due to 
% gravitational effects.
% Returns a vector 6x1, with the corresponding resisitve torque

    theta_0 =  robot.theta + [q;0];
    G = zeros(length(q),1);
    
    for i=1:length(q)
        [Jv, ~]  = ComputeJacGi(robot.alpha, robot.d, theta_0, robot.r, robot.cg, i);
        G = G - robot.m(i)*(Jv'*robot.g);
    end

end
