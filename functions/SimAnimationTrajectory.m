function [] = SimAnimationTrajectory(robot,qdk,X_star,video)
%AnimationTrajectory
% Displays the sequence of frames of the trajectory perfomed by the robot
% and the trajectory to be followed.

    vw = VideoWriter(video);
    open(vw) 
    for k=2:10:length(qdk(1,:))

%         scatter3(X_star(1,1:k),X_star(2,1:k),X_star(3,1:k),1,'MarkerFaceColor','r','MarkerEdgeColor','r','LineWidth',0.05)
%         hold on

        
        g_0Ek = ComputeDGM(robot.alpha, robot.d, [qdk(:,k);0], robot.r);
        grid on
        % Q5
        PlotFrame(g_0Ek)
        hold on
        PlotRobot(robot, qdk(:,k))
        hold off
        legend('Desired Trajectory X_d','Initial Position Xd_i','Final Position Xd_f', 'Perfomed Trajectory' , 'X', 'Y', 'Z')
%         title('Performed Trajectory')
        drawnow
        frame = getframe(gcf);
        writeVideo(vw,frame);
    end
    close(vw)
end