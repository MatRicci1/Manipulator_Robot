function [] = JointTemporalPlot(qmin, qmax, Te, qdk)
%JointTemporalPlot
% Plot the joint angles and their restrictions as
% a function of time

t = (1:length(qdk(1,:)))*Te;

for i=1:6
    subplot(3,2,i)
    y = qdk(i,:);
    plot(t,y, 'LineWidth', 2)
    hold on
    yline = qmin(i)*ones(1,length(qdk(1,:)));
    plot(t,yline, '--r', 'LineWidth', 2)
    hold on
    yline = qmax(i)*ones(1,length(qdk(1,:)));
    plot(t,yline, '--r', 'LineWidth', 2)
    grid on
    legend(['q',num2str(i)],['q',num2str(i),'_{min}'],['q',num2str(i),'_{max}'],'Location','best')
    xlabel('t [s]')
    ylabel('\theta [rad]')
    title(['q_',num2str(i)])
end

end