% This program reads position data from csv and plots it
close all
redc = [0.9 0.1 0.11];
bluec = [0.22 0.5 0.72];
greenc = [0.3 0.69 0.29];
purplec = [0.6 .3 .64];
orangec = [1 .5 0];

data = readtable("square_setpoint_70.csv",'ReadVariableNames', true);

plotting = 335:2:length(data.t)-350;
t = data.t(plotting);
center_x = data.x_center(plotting);
center_y = data.y_center(plotting);
offset_x = data.x_offset(plotting);
offset_y = data.y_offset(plotting);
velocity_u = zeros(length(t),1);
velocity_v = zeros(length(t),1);

for index = 2:3:length(t)
   velocity_u(index) = (center_x(index)-center_x(index-1))/(t(index)-t(index-1));
   velocity_v(index) = (center_y(index)-center_y(index-1))/(t(index)-t(index-1));
end

figure
%plot(center_x+0.03, center_y+0.02,'.','Color', bluec,'MarkerSize',10,'Linewidth',2);
xlim([-.10 .50]);
ylim([-.10 .50]);
xlabel('x [m]');
ylabel('y [m]');
hold on
grid on
box on
plot([0 0 0.40 0.40 0],[0 0.40 0.40 0 0],'Color',redc,LineWidth=2);
%annotation('arrow',[0.60 0.4],[0.16 0.16],'Color',orangec,LineWidth=2);
%plot(offset_x, offset_y,'.','Color', redc,'MarkerSize',10,'Linewidth',2);
%quiver3(center_x,center_y,zeros(length(center_y),1),zeros(length(center_y),1),zeros(length(center_y),1),((velocity_u.^2+velocity_v.^2).^0.5-2)*2, 0.5, 'Color', redc,'LineWidth',1.5,'Alignment','head','ShowArrowHead','on','MaxHeadSize',1)
quiver(center_x+0.03,center_y+0.02,velocity_u,velocity_v, 1, 'Color', bluec,'LineWidth',2,'Alignment','head','ShowArrowHead','on','MaxHeadSize',2);
%plot(t,(velocity_v.^2+velocity_u.^2).^0.5,'.','Color', bluec,'MarkerSize',10,'Linewidth',2)
%plot(t,velocity_u,'.','Color', bluec,'MarkerSize',10,'Linewidth',2)
legend('Setpoint', 'Rover Center Velocity','NumColumns',2,'Location', 'North');
hold off

set(gca, 'LineWidth', 2); % maybe we change to 1.5 but i think2s fine
set(gca, 'FontSize', 14);
set(gca, 'FontWeight', 'bold');
set(gcf,'position',[0,0,500,500]); % change last two to alter size of graph