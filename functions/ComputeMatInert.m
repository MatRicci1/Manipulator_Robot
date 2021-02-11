function A = ComputeMatInert(robot, q)
%ComputeMatInert
% Calculates the inertia matrix of a given configuration
% 
% Returns the inertia matrix of size n_joints x n_joints.

    theta_0 =  robot.theta + [q;0];
%     theta_0 =  robot.theta;
%     theta_0(1:6) = theta_0(1:6) + q;
    A = zeros(6);

    for i=1:length(q)

        [Jvi, Jwi, p_cg_O0, g] = ComputeJacGi(robot.alpha, robot.d, theta_0, robot.r, robot.cg, i);
        IG_i = robot.I(:,:,i) - robot.m(i)*skew(robot.cg(:,i));
        RO_i = g(1:3,1:3);
        IO_i = RO_i*IG_i*RO_i';
        A = A + (robot.m(i)*(Jvi'*Jvi) + Jwi'*IO_i*Jwi);
    end
    A = A + diag(robot.red.^2.*robot.Jm);


function M = skew(x)

    X=[    0, -x(3),  x(2) ;
        x(3),     0, -x(1) ;
       -x(2),  x(1),    0 ];
    M = -X*X;



