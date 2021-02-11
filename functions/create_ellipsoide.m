function [] = create_ellipsoide(P,Sig,U)
%create_ellipsoide
% Calculates the Velocity Manipulability ellipsoide from
% a given joint configuration
% 
% Plot the ellipsoide over the robot 

[X,Y,Z] = ellipsoid(P(1),P(2),P(3),Sig(1,1),Sig(2,2),Sig(3,3));
h = surf(X,Y,Z);
a=[1 0 0];
b=U(:,1);
[theta, w] = inv_Rot(U);
rotate(h,w, theta*180/pi, [P(1),P(2),P(3)]);
alpha 0.3

hold on
quiver3(P(1),P(2),P(3), U(1,1),U(2,1),U(3,1),'LineWidth',2, 'color','m')
hold on
quiver3(P(1),P(2),P(3), U(1,2),U(2,2),U(3,2),'LineWidth',2, 'color','g')
hold on
quiver3(P(1),P(2),P(3), U(1,3),U(2,3),U(3,3),'LineWidth',2, 'color','c')


end
