function [newXv, newYv, newMap, nodes] = bufferMap(tempMap, width, length, mapVertices)
% This is a helper function that buffers the obstacle so that the robot's
% radius can be taken into account
% Input:
%       tempMap     map object with all walls [x1, y1, x2, y2]
%       width       The buffered width
%       length      The buffered length
%       mapVertices The vertices of the boundary
% Output:
%       newXv       Updated xv
%       newYv       Updated yv
%       newMap      a txt file
%       nodes       effective nodes of the new map
%                   NOTE: nodes outside the shrink boundary will not be
%                   included
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the number of obstacles
[wallNum, ~] = size(tempMap);
newXv = zeros(wallNum, 4);
newYv = zeros(wallNum, 4);
newMap = zeros(wallNum, 8);
nodes = [];

for i = 1 : wallNum
    dx = tempMap(i, 3) - tempMap(i, 1);
    dy = tempMap(i, 4) - tempMap(i, 2);
    normLength = sqrt(dx^2 + dy^2);
    nx = -dy / normLength;
    ny = dx / normLength;
    tx = dx / normLength;
    ty = dy / normLength;

    % Calculate the displacement
    dxHalf = (width * nx) / 2;
    dyHalf = (width * ny) / 2;
    dxEndpoint = (length * tx) / 2;
    dyEndpoint = (length * ty) / 2;

    % Calculate the bloated wall coordinates
    x1 = tempMap(i, 1) - dxHalf - dxEndpoint;
    y1 = tempMap(i, 2) - dyHalf - dyEndpoint;
    [in, on] = inpolygon(x1, y1, mapVertices(:, 1), mapVertices(:, 2));
    if in
        nodes = [nodes; [x1, y1, i]];
    end  
    
    x2 = tempMap(i, 1) + dxHalf - dxEndpoint;
    y2 = tempMap(i, 2) + dyHalf - dyEndpoint;
    [in, on] = inpolygon(x2, y2, mapVertices(:, 1), mapVertices(:, 2));   
    if in
        nodes = [nodes; [x2, y2, i]];
    end
    
    x3 = tempMap(i, 3) + dxHalf + dxEndpoint;
    y3 = tempMap(i, 4) + dyHalf + dyEndpoint;
    [in, on] = inpolygon(x3, y3, mapVertices(:, 1), mapVertices(:, 2));    
    if in
        nodes = [nodes; [x3, y3, i]];
    end
    
    x4 = tempMap(i, 3) - dxHalf + dxEndpoint;
    y4 = tempMap(i, 4) - dyHalf + dyEndpoint;
    [in, on] = inpolygon(x4, y4, mapVertices(:, 1), mapVertices(:, 2));
    if in
        nodes = [nodes; [x4, y4, i]];
    end
    
    % Return the bloated wall
    newXv(i, :) = [x1, x2, x3, x4];
    newYv(i, :) = [y1, y2, y3, y4];
    newMap(i, :) = [x1, y1, x2, y2, x3, y3, x4, y4];
end

