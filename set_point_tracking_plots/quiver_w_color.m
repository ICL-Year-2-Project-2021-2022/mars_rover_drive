% This program reads position data from csv and plots it
close all
redc = [0.9 0.1 0.11];
bluec = [0.22 0.5 0.72];
greenc = [0.3 0.69 0.29];
purplec = [0.6 .3 .64];
orangec = [1 .5 0];

data = readtable("square_setpoint_70.csv",'ReadVariableNames', true);

plotting = 320:4:length(data.t)-350;
t = data.t(plotting);
center_x = data.x_center(plotting);
center_y = data.y_center(plotting);
offset_x = data.x_offset(plotting);
offset_y = data.y_offset(plotting);
velocity_u = zeros(length(t),1);
velocity_v = zeros(length(t),1);

for index = 2:1:length(velocity_u)
   velocity_u(index) = (center_x(index)-center_x(index-1))/(t(index)-t(index-1));
   velocity_v(index) = (center_y(index)-center_y(index-1))/(t(index)-t(index-1));
end

X = center_x;
Y = center_y;
U = velocity_u;
V = velocity_v;
Z = (velocity_u.^2+velocity_v.^2).^0.5;


% Demo data
%[X,Y] = meshgrid(-2:0.2:2);
%Z = X .* exp(-X.^2 - Y.^2);
%[U,V] = gradient(Z,0.2,0.2);

% Create axes
ax = axes(); 
hold(ax,'on')
box(ax,'on')

% Define the colormap for the quiver arrows.
% cmap can have any number of rows.
cmap = hot(255); 
ax.Colormap = cmap; 

% Assign colors based on magnitude of vectors
vectorMagnitude = hypot(U(:),V(:)); 
% Scale magnitudes to rows of colormap
%vectorMagnitude(vectorMagnitude<2) = 2;
vecMagNorm = (vectorMagnitude-min(vectorMagnitude))./range(vectorMagnitude);
%vecMagNorm = (vectorMagnitude-2.0)./range(vectorMagnitude);
vecColorIdx = round(vecMagNorm * (size(cmap,1)-1)) + 1; 

% Plot the quiver data
for i = 1:numel(Z)
    quiver(ax, X(i),Y(i),U(i),V(i), 0.1, ...
        'Color', cmap(vecColorIdx(i),:),'LineWidth',2,'Alignment','head','ShowArrowHead','on','MaxHeadSize',1)
end

% Set properties for the main axes
axis equal
xlim(ax, [-0.1 0.5])
ylim(ax, [-0.1 0.5])

% Add colorbar
cb = colorbar(ax); 
% Set colorbar range
caxis(ax, [floor(vectorMagnitude(1)), ceil(vectorMagnitude(2))])
%caxis(ax, [2, 2.3])
% Label the colorbars
ylabel(cb,'Vector magnitude')