function [Jv, Jw, p_cg_O0, g] = ComputeJacGi(alpha, d, theta, r, cg, ji)
%ComputeJacGi
% Calculates the jacobian matrix of the centre of mass
% of a rigid body
%
% Returns the jacobian matrix of size ji x ji.

    alpha_cg = alpha(1:ji);
    d_cg = d(1:ji);
    theta_cg = theta(1:ji);
    r_cg = r(1:ji);

    g = ComputeDGM(alpha_cg,d_cg,theta_cg,r_cg);
    p_cg_O0 = g(1:3,:)*[cg(:,ji);1];

    J = zeros(6);
    J(:,1:ji) = ComputeJac(alpha_cg,d_cg,theta_cg,r_cg, p_cg_O0,ji);

    Jv = J(1:3,:);
    Jw = J(4:6,:);


end