function m = plotSquareMap(map)
% This function plots the map with a given map file
% Input:        map:    a k by 4 matrix

[k, colNum] = size(map);
xCoordinates = ones(k, 2);
yCoordinates = ones(k, 2);

for i = 1 : k
    xCoordinates(i, :) = [map(i, 1), map(i, 3)];
    yCoordinates(i, :) = [map(i, 2), map(i, 4)];
end

l = plot(xCoordinates(1, :), yCoordinates(1, :), 'HandleVisibility','off');
l.Color = 'k';
l.LineWidth = 1.5;

hold on
for i = 2 : k
    m = plot(xCoordinates(i, :), yCoordinates(i, :), 'HandleVisibility','off');
    m.Color = 'k';
    m.LineWidth = 1.5;
end
hold off
end