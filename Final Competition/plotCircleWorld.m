function [a, h] = plotCircleWorld(map)
% This function takes in a map and plots the circle world
% Input:
%           map     K by 3 [centerx centery radius]

% map = load('hw6Sphere.txt');
axis equal
a = viscircles([map(1, 1:2)], map(1, 3), 'Color', 'k');
[row, col] = size(map);
for i = 2 : row
    hold on
    h = viscircles([map(i, 1:2)], map(i, 3), 'Color', '#EDB120');
end

% hold on
hold off
title('The Sphere World');
xlabel('Global X Coordinate (m)');
ylabel('Global Y Coordinate (m)');
% legend([a, h, k], 'Map Boundary', 'Obstacles', 'Simulator World');

