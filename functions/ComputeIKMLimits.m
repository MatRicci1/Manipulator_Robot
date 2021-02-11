function [X_star,theta_star, q_star] = ComputeIKMLimits(robot, Xd_i, Xd_f, V, Te, q_i, q_min, q_max, k_max, eps_x, tradeoff, method)
%ComputeIKMLimits
% Calculates the best points and joint configurations of the desired
% trajectory while respecting the angle restrictions qmax and qmin
    
    n = length(q_i);
    delta_X = Xd_f - Xd_i;
    dist = norm(delta_X);
    u_x = (Xd_f - Xd_i)/dist;

    temp = dist/V;
    it=  ceil(temp/Te);
    t_lim = 0:Te:temp;
    Xdk_dot = V*u_x;
    q_star= zeros(length(q_i),it);
    q_star_dot = zeros(length(q_i),it);
    X_star = zeros(3,it);
    % iter 1

    [Xdk, thetadk, qdk] = ComputeIKM(robot, Xd_i, Xd_f, V, Te, q_i, k_max, eps_x, method);
    qi_dash =  (q_max + q_min)/2;


    for k=1:it
        dH_lim = 2*(qdk(:,k)-qi_dash)./((q_max - q_min).^2);
        q0_dot = -tradeoff*dH_lim;
        
        J = ComputeJac(robot.alpha, robot.d, thetadk(:,k), robot.r, Xdk(:,k), robot.n_joints);
        rank_J(k) = rank(J);
        Jv =  J(1:3,:);
        J_sharp = pinv(Jv);
        N_j = eye(length(Jv(1,:))) - (J_sharp*Jv);
        q_star_dot(:,k) = J_sharp*Xdk_dot + N_j*q0_dot;
    end
    
    for i=1:n
        q_star(i,:) = cumtrapz(t_lim,q_star_dot(i,:));
    end
    
    q_star(:,:) = q_star(:,:) + q_i;
    
    theta_star = [q_star;zeros(1,it)] + robot.theta;

    for k=1:it
        g = ComputeDGM(robot.alpha, robot.d, theta_star(:,k), robot.r);
        X_star(:,k) = g(1:3,4);
    end

end
