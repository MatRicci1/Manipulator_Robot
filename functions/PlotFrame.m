function [] = PlotFrame(g)
%PlotFrame
% Plot reference frame and chosen frame with their
% respective axes   
    
    % Reference frame R0
    c = 0.3;
    c2 = 0.3;
    R0_x0 = c*[0, 0, 0];
    R0_x1 = c*[1, 0, 0];

    R0_y0 = c*[0, 0, 0];
    R0_y1 = c*[0, 1, 0];

    R0_z0 = c*[0, 0, 0];
    R0_z1 = c*[0, 0, 1];
    
    % end-effector frame Re
    Rn_x0 = g(1:3,4);
    Rn_x1 = c2*g(1:3,1);

    Rn_y0 = g(1:3,4);
    Rn_y1 = c2*g(1:3,2);

    Rn_z0 = g(1:3,4);
    Rn_z1 = c2*g(1:3,3);
    
    quiver3(R0_x0(1),R0_x0(2),R0_x0(3), R0_x1(1),R0_x1(2),R0_x1(3),'LineWidth',2, 'Color','r','AutoScale','off')
    hold on
    quiver3(R0_y0(1),R0_y0(2), R0_y0(3), R0_y1(1),R0_y1(2),R0_y1(3),'LineWidth',2, 'color','b','AutoScale','off')
    hold on
    quiver3(R0_z0(1),R0_z0(2),R0_z0(3), R0_z1(1),R0_z1(2),R0_z1(3),'LineWidth',2, 'color','g','AutoScale','off')
    hold on
    
    quiver3(Rn_x0(1),Rn_x0(2),Rn_x0(3), Rn_x1(1),Rn_x1(2),Rn_x1(3),'LineWidth',2, 'color','r','AutoScale','off','HandleVisibility', 'off')
    hold on
    quiver3(Rn_y0(1),Rn_y0(2),Rn_y0(3), Rn_y1(1),Rn_y1(2),Rn_y1(3),'LineWidth',2, 'color','b', 'AutoScale','off','HandleVisibility', 'off')
    hold on
    quiver3(Rn_z0(1),Rn_z0(2),Rn_z0(3), Rn_z1(1),Rn_z1(2),Rn_z1(3),'LineWidth',2, 'color','g', 'AutoScale','off','HandleVisibility', 'off')
    hold on

    %plot3([R0_x0(1);Rn_x0(1)],[R0_y0(2);Rn_y0(2)],[R0_z0(3);Rn_z0(3)],'c--')
    axis equal

    hold off
    
end



