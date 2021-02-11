function [] = simdata_plot_max_min(max,min,sim_data, plot_title)
%simdata_plot 
% Plot the data from simulink of the 6 joints
% as a function of time

for i=1:6
    subplot(3,2,i)
    t = sim_data{i}.Values.Time;
    plot(sim_data{i}.Values,'LineWidth', 2)
    hold on
    yline = min(i)*ones(1,length(t));
    plot(t,yline, '--r', 'LineWidth', 2)
    hold on
    yline = max(i)*ones(1,length(t));
    plot(t,yline, '--r', 'LineWidth', 2)
    grid on
    grid on
    legend([plot_title,num2str(i)],'Location','best')
    xlabel('t [s]')
    ylabel('\theta [rad]')
    title([plot_title,num2str(i)])
end



end

