function [theta, w] = inv_Rot(R)
%inv_Rot Inverse relationship of Rodrigues formula
%   Identifcation of the axis of rotation w and theta (0<theta<pi)
    
    theta = atan2(0.5*sqrt((R(3,2)-R(2,3))^2 + (R(1,3)-R(3,1))^2 + (R(2,1) - R(1,2))^2),...
                  0.5*(R(1,1) + R(2,2) + R(3,3) - 1));
    
    tol = 1e-3;
    if (0 < sin(theta)) && (sin(theta)< tol)
        w = [0;0;0];
        theta = 0;
    else
        w = [(R(3,2)-R(2,3));
             (R(1,3)-R(3,1)) ;
             (R(2,1)-R(1,2))]./(2*sin(theta));
    end
