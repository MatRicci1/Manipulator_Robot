function  [Xdk, thetadk, qdk] = ComputeIKM(robot, Xd_i, Xd_f, V, Te, q_i, k_max, eps_x, method)
%ComputeIKM
% Calculates the best points and joint configurations of the desired
% trajectory   
    
    delta_X = Xd_f - Xd_i;
    dist = norm(delta_X);
    u_x = (Xd_f - Xd_i)/dist;

    temp = dist/V;
    it=  ceil(temp/Te);
    Xdk = zeros(length(Xd_i),it);
    qdk = zeros(length(q_i),it);
    
    % iter 1
    Xdk(:,1) = Xd_i;
    qdk(:,1) = q_i;

    
    
    for i=2:it-1
        Xdk(:,i) = Xdk(:,i-1) + (V*Te)*u_x;
        [~, ~, qdk(:,i)] = ComputeIGM(robot, Xdk(:,i), qdk(:,i-1), k_max, eps_x, method);

    end
    
    Xdk(:,it) = Xd_f;
    [~, ~, qdk(:,it)] = ComputeIGM(robot, Xdk(:,it), qdk(:,it-1), k_max, eps_x, method);

    thetadk = [qdk;zeros(1,it)] + robot.theta;
    
end