function J = ComputeJac(alpha, d, theta, r, p_e, n_joints)
%ComputeJac
% Calculates the jacobian matrix of a given point
%
% Returns the jacobian matrix of size n_joints x n_joints.

    J = zeros(6,n_joints);
    g_0N = eye(4);
    % Khalil-Kleinfinger Convention
    Zi = [0; 0; 1];

    for i=1:n_joints
        g = TransformMatElem(alpha(i), d(i), theta(i), r(i));
        g_0N = g_0N*g;
        
        % Revolution Joint
        R_0i = g_0N(1:3,1:3);
        p_in = g_0N(1:3,4);
        p = p_e - p_in;

        J(:,i) = [cross(R_0i*Zi,p);
                  R_0i*Zi;];
    end
    
end