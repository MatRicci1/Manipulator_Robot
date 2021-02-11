function [] = PlotRobot(robot, q)
%PlotRobot
% Plot robot's rigid bodies and joints.       
    
    g = eye(4);
    theta_i = [q;0] + robot.theta;
    for i = 1:(robot.n_joints+1)
        if i == robot.n_joints+1 %from g0N to g0E, theta = 0
            g = g * TransformMatElem(robot.alpha(i),robot.d(i),theta_i(i),robot.r(i));
        else
            g = g * TransformMatElem(robot.alpha(i),robot.d(i),theta_i(i),robot.r(i));
        end

        ps(:,i) = g(1:3,4);
        if i == 1 % Draw the line from o_0 to o_1
            plot3([0, ps(1,i)],[0, ps(2,i)],[0, ps(3,i)],...
                'k','LineWidth',3,'HandleVisibility','off');
            %scatter3(ps(1,i),ps(2,i),ps(3,i), 8,'MarkerFaceColor','k', 'LineWidth', 3,'HandleVisibility', 'off')
        else % Draw the line from o_i-1 to o_i
            plot3([ps(1,i-1), ps(1,i)],[ps(2,i-1), ps(2,i)],[ps(3,i-1), ps(3,i)],...
                'k','LineWidth',3,'HandleVisibility','off');
            scatter3(ps(1,i-1),ps(2,i-1),ps(3,i-1), 8,'MarkerFaceColor','k','MarkerEdgeColor','k', 'LineWidth', 3,'HandleVisibility', 'off')
        end
    end