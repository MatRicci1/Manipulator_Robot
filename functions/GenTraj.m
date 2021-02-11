function qc  = GenTraj(qdi, qdf,t, tf_min)
%GENTRAJ 
%   Generates a 5th degree polynomial trajectory from 
%   the initial and final values qdi and qdf.

D = qdf - qdi;
t_star = t/tf_min;
r = 6*t_star^5 -15*t_star^4 +10*t_star^3;
qc = qdi + r*D;
end