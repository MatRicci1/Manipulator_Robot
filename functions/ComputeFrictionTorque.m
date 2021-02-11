function Gamma_f =  ComputeFrictionTorque(robot,q_dot)
%ComputeFrictionTorque
% Computes the Friction Torque from the viscous frictions
% and the current speed 
% Returns a vectir Gamma_f 6x1.

    Gamma_f = diag(q_dot)*robot.Fv;
end