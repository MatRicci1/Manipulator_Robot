function [Xd_estimated, err, q_star] = ComtputeIGM(robot, Xd, q_0, k_max, eps_x, method)
%ComputeIGM
% Calculates the estimated position from a desired position 
% and an initial joint configuration by an iterative method
%
% Returns the estimated position, the error and the estimated
% joint configuration   
    
    % tic
    step_IGM = 2;
    it = 1;
    err = 5000;
    f = [0,0,0]';
    while (it < k_max && err > eps_x)
        theta_0 = [q_0;0] + robot.theta;
        g = ComputeDGM(robot.alpha, robot.d, theta_0, robot.r);
        Xd_estimated = g(1:3,4);
        % J_full = ComputeJac(robot.alpha, robot.d, robot.theta + q_0, robot.r, Xd);
        J_full = ComputeJac(robot.alpha, robot.d,  theta_0, robot.r, Xd, robot.n_joints);
        J = J_full(1:3,:);

        if strcmp(method, 'NewtonRaphson')
            J_star = pinv(J);  
        end
        if strcmp(method, 'Gradient')
            J_star =  step_IGM*J';  
        end
        q_k = q_0 + J_star*(Xd-Xd_estimated);
        err = norm(Xd-Xd_estimated);
        it = it + 1;

        q_0 = q_k;
    end
    
    
    q_star = q_0;
    % fprintf("Method %s: ",method); toc
end