cla;

x = categorical(["Blue", "Green", "Red", "Orange", "Violet", "White", "Indigo", "Yellow"]);
x = reordercats(x,["Blue", "Green", "Red", "Orange", "Violet", "White", "Indigo", "Yellow"]);
data = [71.95, 65.78,65.3, 59.24,58.72, 56.74, 56.3, 56];
stderr = [10.67, 25.61, 4.2, 7.67, 11.1, 6.34, 28.72, 7.06];

b= bar(x,data);                
b.FaceColor = 'flat';
b.CData(1,:) = [0 0 1];
b.CData(2,:) = [0 1 0];
b.CData(3,:) = [1 0 0];
b.CData(4,:) = [255/255 169/255 0];
b.CData(5,:) = [238/255 130/255 238/255];
b.CData(6,:) = [1 1 1];
b.CData(7,:) = [75/255 0 130/255];
b.CData(8,:) = [1 1 0];

title('Optical Flow Sensor Effectiveness by Color of LED');
ax = gca;
ax.FontSize = 16;

xlabel('Color of LED');
ylabel('Average Sensor Quality on Featureless Mars Surface (S-Qual)') ;

hold on

er = errorbar(x,data,stderr,stderr);    
er.Color = [0 0 0];                            
er.LineStyle = 'none';  
er.LineWidth = 1.5;

hold off