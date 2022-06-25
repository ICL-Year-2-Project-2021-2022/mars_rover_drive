% This program reads position data from csv and plots it
close all
redc = [0.9 0.1 0.11];
bluec = [0.22 0.5 0.72];
greenc = [0.3 0.69 0.29];
purplec = [0.6 .3 .64];
orangec = [1 .5 0];

figure
xlabel('PWM duty cycle [%]');
ylabel('Velocity [m/s]');
hold on
grid on
box on

plot(motor_profile(:,1), motor_profile(:,2)/100,'LineWidth',2,'Color',redc,'Marker','.','MarkerSize',20);

%legend('Setpoint', 'Rover Center Velocity','NumColumns',2,'Location', 'North');
hold off

set(gca, 'LineWidth', 2); % maybe we change to 1.5 but i think2s fine
set(gca, 'FontSize', 14);
set(gca, 'FontWeight', 'bold');
set(gcf,'position',[0,0,600,400]); % change last two to alter size of graph