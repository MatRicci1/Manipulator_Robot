clear all
close all
clc
addpath('functions')
%%%%%%%%%%%%%%%%%%%%%%%%%



%% Problem specifications
%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%
% Geometric Parameters
%%%%%%%%%%%%%%%%%%%%%%%%% 

d3   = 0.7;
r1   = 0.5;
r4   = 0.2;
r_e  = 0.1;

%%%%%%%%%%%%%%%%%%%%%
% Analyse Parameters
%%%%%%%%%%%%%%%%%%%%%

% Joint configuration qi
qi = [-pi/2, 0, -pi/2, -pi/2, -pi/2, -pi/2]';

% Joint configuration qf
qf = [0, pi/4, 0, pi/2, pi/2, 0]';

% Joint velocity q_dot
q_dot = [0.5, 1, -0.5, 0.5, 1, -0.5]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 2 - Robot Reference Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot.qmin = [-pi, -pi/2, -pi,  -pi, -pi/2,   -pi]';
robot.qmax = [  0,  pi/2,   0, pi/2,  pi/2,  pi/2]';

robot.n_joints = 6;
robot.n_frames = 7;
robot.alpha = [0, pi/2, 0, pi/2, -pi/2, pi/2, 0]';
robot.d     = [0, 0, d3, 0, 0, 0, 0]';
robot.theta = [0, 0, 0 + pi/2, 0, 0, 0, 0]';
robot.r     = [r1, 0, 0, r4, 0, 0, r_e]';

% Mass of each arm of the robot
robot.m = [15.0, 10.0, 1.0, 7.0, 1.0, 0.5]';

% Robot's Joints Inertia Matrix
robot.I = zeros(3,3,robot.n_joints);
robot.I(:,:,1) = [0.80, 0.00, 0.05 ;
                  0.00, 0.80, 0.00 ;  % Inertial tensor of the body 1 
                  0.05, 0.00, 0.10];  % in the R1 frame[kg*m^2]

robot.I(:,:,2) = [0.10, 0.00, 0.10 ;
                  0.00, 1.50, 0.00 ;  % Inertial tensor of the body 2
                  0.10, 0.00, 1.50];  % in the R2 frame[kg*m^2]
  
robot.I(:,:,3) = [0.05, 0.00, 0.00 ;
                  0.00, 0.01, 0.00 ;   % Inertial tensor of the body 3 
                  0.00, 0.00, 0.05];   % in the R3 frame[kg*m^2]
  
robot.I(:,:,4) = [0.50, 0.00, 0.00 ;
                  0.00, 0.50, 0.00 ;  % Inertial tensor of the body 4 
                  0.00, 0.00, 0.05];  % in the R4 frame[kg*m^2]
  
robot.I(:,:,5) = [0.01, 0.00, 0.00 ;
                  0.00, 0.01, 0.00 ;  % Inertial tensor of the body 5 
                  0.00, 0.00, 0.01];  % in the R5 frame[kg*m^2]
  
robot.I(:,:,6) = [0.01, 0.00, 0.00 ;
                  0.00, 0.01, 0.00 ;  % Inertial tensor of the body 6 
                  0.00, 0.00, 0.01];  % in the R5 frame[kg*m^2]
    

% Vectors of Gi given in frame Ri 
robot.cg = [ 0.00, 0.35,  0.00, 0.00, 0.00, 0.00 ;
             0.00, 0.00, -0.10, 0.00, 0.00, 0.00 ; % Coordinates of Gi given  
            -0.25, 0.00,  0.00, 0.00, 0.00, 0.00]; % in frame Ri [m]

% Moment of inertia of the actuator rotor [kg*m^2]
robot.Jm = (10^-5)*[1; 1; 1; 1; 1; 1]; 

% Reduction ratio
robot.red = [100; 100; 100; 70; 70; 70]; % [ ]

% Joint viscous frictions 
robot.Fv = [10; 10; 10; 10; 10; 10]; % [N*m*(rad^-1)*s]

% Maximal motor torques
robot.tau_max = [5; 5; 5; 5; 5; 5];  % [N*m]

%Acceleration of gravity in the reference frame
robot.g = [0.00; 0.00; -9.81];  % [m*s^-2]
%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Problem design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Questions 3 and 4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Analyse position
theta_i = robot.theta + [qi;0];
theta_f = robot.theta + [qf;0];

% Transformation Matrix 
g_0Ei = ComputeDGM(robot.alpha, robot.d, theta_i, robot.r);
g_0Ef = ComputeDGM(robot.alpha, robot.d, theta_f, robot.r);

% Position Vector of End frame at qi and qf
Pi = g_0Ei(1:3,4);
Pf = g_0Ef(1:3,4);

% Rotation Matrix of End frame at qi and qf
Rot_i = g_0Ei(1:3,1:3);
Rot_f = g_0Ef(1:3,1:3);

% Unitary vector and angle of the transformation
[phi_i,n_i] = inv_Rot(Rot_i);
[phi_f,n_f] = inv_Rot(Rot_f);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Jacobian of End frame

% Joint configuration qi
J0_Ei = ComputeJac(robot.alpha, robot.d, theta_i, robot.r, Pi, robot.n_joints);
% Joint configuration qf
J0_Ef = ComputeJac(robot.alpha, robot.d, theta_f, robot.r, Pf, robot.n_joints);

% Velocties of end frame in i and f
V0_0Ei = J0_Ei*q_dot;
V0_0Ef = J0_Ef*q_dot;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Single value decomposition
[U_i, Sig_i, V_i] = svd(J0_Ei(1:3,:));
[U_f, Sig_f, V_f] = svd(J0_Ef(1:3,:));

% Velocity Manipulability
Vel_Man_i = prod(diag(Sig_i));
Vel_Man_f = prod(diag(Sig_f));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Joint configuration qi
Xd_i = [-0.1, -0.7, 0.3]';
q0_i = [-1.57, 0.00, -1.47, -1.47, -1.47, -1.47]';

% Joint configuration qf
Xd_f = [0.64, -0.1, 1.14]';
q0_f = [0, 0.80, 0.00, 1.00, 2.00, 0.00]';

% Max number of iterations 
k_max = 100;
% Max error
eps_max = 0.001; 

% IGM by Newton-Raphson Method
[f_in, err_in, q_star_in] = ComputeIGM(robot, Xd_i, q0_i, k_max, eps_max, 'NewtonRaphson');
[f_fn, err_fn, q_star_fn] = ComputeIGM(robot, Xd_f, q0_f, k_max, eps_max, 'NewtonRaphson');

% Gradient Method
[f_ig, err_ig, q_star_ig] = ComputeIGM(robot, Xd_i, q0_i, k_max, eps_max, 'Gradient');
[f_fg, err_fg, q_star_fg] = ComputeIGM(robot, Xd_f, q0_f, k_max, eps_max, 'Gradient');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 9 and 10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Linear velocity 
V_d = 1;
% Time sample
Te = 0.001;

% IKM
[Xdk, thetadk, qdk] = ComputeIKM(robot, Xd_i, Xd_f, V_d, Te, qi, k_max, eps_max, 'NewtonRaphson');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 11
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% IKMLimits for tradeoff = 0.5
% tradeoff = 0.5;
% [X_star, theta_star, q_star] = ComputeIKMLimits(robot, Xd_i, Xd_f, V_d, Te, qi, robot.qmin, robot.qmax, k_max, eps_max, tradeoff, 'NewtonRaphson');
% err_tradeoff_05 = norm(X_star - Xdk);

% IKMLimits for tradeoff = 1
tradeoff = 1;
[X_star, theta_star, q_star] = ComputeIKMLimits(robot, Xd_i, Xd_f, V_d, Te, qi, robot.qmin, robot.qmax, k_max, eps_max, tradeoff, 'NewtonRaphson');
err_tradeoff_10 = norm(X_star - Xdk);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 12
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Joint_test_JacGi= 6;

% Jacobian matrix of centre of mass of rigid body
[Jv, Jw, p_cg_O0, g] = ComputeJacGi(robot.alpha, robot.d, [qi;0] + robot.theta, robot.r, robot.cg, Joint_test_JacGi);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inertia matrix
Ai = ComputeMatInert(robot, qi);
Af = ComputeMatInert(robot, qf);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 14
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Min and Max values for the inertia matrix
[mu_2,mu_1] =  A_bounds(robot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 15
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Torque due to the gravity
Gi = ComputeGravTorque(robot, qi);
Gf = ComputeGravTorque(robot, qf);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 16
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Max value of torque due to the gravity
gb = G_bounds(robot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Torque due to joint friction
Gamma_f =  ComputeFrictionTorque(robot,q_dot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial and final cofigurations
robot.qd_i = [-1; 0; -1; -1; -1; -1];   % [rad] Initial position
robot.qd_f = [ 0; 1;  0;  0;  0;  0];   % [rad] Final position

% Minimal time to follow trajectory
Dj = robot.qd_f - robot.qd_i;
k_aj = robot.tau_max.*robot.red/mu_2;
tf_min = max( sqrt( 10*abs(Dj)./(sqrt(3)*k_aj) ));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Trajactory generation
%qc = GenTraj(qdi, qdf,t, tf_min);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Question 20
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Control parameters
Kp = [5000; 8000; 5500; 800; 5500; 2500];
Kd = [1000; 1000; 500; 100; 100; 500];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulink
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

open_system('main_simulink');
sim('main_simulink.slx');










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
figure(1)
PlotFrame(g_0Ei) % Q5
hold on
create_ellipsoide(Pi,Sig_i,U_i) % Q7
PlotRobot(robot,qi)
legend('X','Y','Z','Velocity Ellipsoid','axis ellipsoid \sigma_1','axis ellipsoid \sigma_2','axis ellipsoid \sigma_3')
title('Ellipsoid of Velocity Manipulabity q_i')


%%
figure(2)
PlotFrame(g_0Ef) % Q5
hold on
create_ellipsoide(Pf,Sig_f,U_f) % Q7
PlotRobot(robot,qf)
legend('X','Y','Z','Velocity Ellipsoid','axis ellipsoid \sigma_1','axis ellipsoid \sigma_2','axis ellipsoid \sigma_3')
title('Ellipsoid of Velocity Manipulabity q_f')


%%  Question 9
figure(3)
title('Performed Trajectory qdk')
AnimationTrajectory(robot,thetadk,qdk,Xd_i,Xd_f,Xdk,'q9.avi')


%% Question 10
figure(4)
suptitle('Time Evolution of q_i')
JointTemporalPlot(robot.qmin, robot.qmax, Te, qdk)

%% Question 11
figure(5)
suptitle('Time Evolution of q_i with Secondary Task')
JointTemporalPlot(robot.qmin, robot.qmax, Te, q_star)

figure(6)
title('Performed Trajectory with Secondary Task')
AnimationTrajectory(robot,theta_star,q_star,Xd_i,Xd_f,X_star,'q11.avi')
%% Q19
figure(7)
suptitle('Desired Joint Trajectory Point qc(t)')
simdata_plot(sim_qc, 'q_c_')
%saveas(gcf,'q19_qc.png')

%% Q20
figure(8)
tau_max = robot.tau_max .* robot.red;
tau_min = - robot.tau_max .* robot.red;
suptitle('Control Joint Torques \Gamma_i(t)')
simdata_plot_max_min(tau_max,tau_min,sim_Gamma, '\Gamma_')
%saveas(gcf,'q20_Gamma.png')

figure(9)
suptitle('Evolution of Joint Trajectories q_i(t)')
simdata_plot(sim_q, 'q_')
%saveas(gcf,'q20_q.png')

figure(10)
suptitle('Evolution of Tracking Errors e(t)')
simdata_plot(sim_error, 'e_')
%saveas(gcf,'q20_error.png')

figure(11)
suptitle('Evolution of q_i(t) and q_c(t)')
simdata_plot(sim_qc, 'qc_')
hold on
simdata_plot(sim_q, 'q_')
%saveas(gcf,'q20_qc_q.png')
