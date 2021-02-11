function [g_0N] = ComputeDGM(alpha, d, theta, r)
%ComputeDGM
% Calculates the succesive transformations from the 
% Reference Frame R0 to another Reference Frame Rn
% 
% Returns the transformation matrix g_0N, 4x4.

    n_frames = length(alpha);
    g_0N = eye(4);
    for i=n_frames:-1:1
        g = TransformMatElem(alpha(i), d(i), theta(i), r(i));
        g_0N = g*g_0N;
    end
end
