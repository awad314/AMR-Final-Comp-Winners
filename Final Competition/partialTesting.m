%%%%%%%%%%%%%%%%%%%
% Map Midpoint Test
%%%%%%%%%%%%%%%%%%%

robotRadius = 0.2;

% Wall offset
off = robotRadius + 0.1;
% Get the data
data = load('practiceMap_4credits_2023.mat');

% an n × 4 matrix consisting of known walls in the environment
map = data.map;
mapBoundary = map(1:4, :);

map = map(5:end, :);
[n, ~] = size(map);

% an m × 4 matrix consisting of optional walls
optWalls = data.optWalls;
[m, ~] = size(optWalls);
tempMap = [map; optWalls];

optNodes = zeros(2*m, 3);

for i = 1 : m
    currWall = optWalls(i, :);
    midPt = [0.5 * (currWall(1, 1) + currWall(1, 3)), 0.5 * (currWall(1, 2) + currWall(1, 4))];
    wallLength = norm(currWall(1, [1, 2]) - currWall(1, [3, 4]));
    orthoX = - (currWall(1, 4) - currWall(1, 2)) / wallLength;
    orthoY = (currWall(1, 3) - currWall(1, 1)) / wallLength;
    offPt1 = midPt + (off + 0.05).* [orthoX, orthoY];
    offPt2 = midPt - (off + 0.05).* [orthoX, orthoY];
    optNodes(2*i - 1, :) = [offPt1, i]; 
    optNodes(2*i, :) = [offPt2, i];
end

% a k × 2 matrix of k waypoints given as [x, y]
waypoints = data.waypoints;
[k, ~] = size(waypoints);
% Construct a new set of waypoints where optional wall waypoints can be
% included; the 3rd column shows if the point is waypoint or wall
waypoints = horzcat(waypoints, zeros(k, 1));

% j × 2 matrix of j waypoints given as [x, y]
ECwaypoints = data.ECwaypoints;
[j, ~] = size(ECwaypoints);
ECwaypoints = horzcat(ECwaypoints, -1 * ones(j, 1));

allWaypoints = [waypoints; ECwaypoints; optNodes];

% an b × 3 matrix where each row gives [tagN um, x, y] for a single beacon
beaconLoc = data.beaconLoc;
[b, ~] = size(beaconLoc);
figure
plotSquareMap(map);
factor = 50;
% hold on
% scatter(optNodes(:, 1), optNodes(:, 2));
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% factor = 50;
% %%%%%%%%%%%%%%%%%%%%%
% % Buffer Wall Testing
% %%%%%%%%%%%%%%%%%%%%%
% 
% % Extract the x and y vertices for inpolygon function
% 
% % Shrink the map boundary inward by fac
% mapVertices = mapBoundary(:, [1, 2]);
% poly = polyshape(mapVertices(:, 1), mapVertices(:, 2));
% mapVertices = polybuffer(poly, -off, "JointType","square").Vertices;
% 
% % Buffer the rest of the walls in the map and get new nodes
% [newXv, newYv, newMap, nodes] = bufferMap(tempMap, off, mapVertices);
% figure
% axis equal
% scatter(nodes(:, 1), nodes(:, 2));
% % Get the basic visibility roadmap

mapVerticesTemp = mapBoundary(:, [1, 2]);
centroid = mean(mapVerticesTemp);
translated_vertices = mapVerticesTemp - centroid;
angles = atan2(translated_vertices(:,2), translated_vertices(:,1));
[~, sorted_indices] = sort(angles);
mapVertices = mapVerticesTemp(sorted_indices, :);
[newXv, newYv, newMap, nodes] = bufferMap(tempMap, off, off+0.05, mapVertices);

hold on
plot([mapVertices(1, 1), mapVertices(2, 1)], [mapVertices(1, 2), mapVertices(2, 2)]);
hold on
plot([mapVertices(2, 1), mapVertices(3, 1)], [mapVertices(2, 2), mapVertices(3, 2)]);
hold on
plot([mapVertices(3, 1), mapVertices(4, 1)], [mapVertices(3, 2), mapVertices(4, 2)]);
hold on
plot([mapVertices(4, 1), mapVertices(1, 1)], [mapVertices(4, 2), mapVertices(1, 2)]);
[edgeMatrix, edgesPath] = createRoadmap(newXv, newYv, nodes, factor);
% 
% % Start from the nearest next waypoint
% [iterNum, ~] = size(allWaypoints);
% path = [];
% weightDis = intmax;
% 
% start = allWaypoints(1, [1, 2]);
% % Select the first waypoint to visited -- Least cost
% for i = 2 : iterNum
%     [pathNodes, d] = findPath(nodes, edgeMatrix, start, allWaypoints(i, [1, 2]), newXv, newYv, factor);
%     if d < weightDis
%         goal = allWaypoints(i, [1, 2]);
%         weightDis = d;
%         path = pathNodes;
%     end
% end
% 
% path

% 
% %%%%%%%%%%%%%%%%%%%%%%
% % Shrink Boundary Test
% %%%%%%%%%%%%%%%%%%%%%%
% distance = -0.5;
% vertices = mapBoundary(:, [1, 2]);
% poly = polyshape(vertices(:, 1), vertices(:, 2));
% 
% shrunk_vertices = polybuffer(poly, distance, "JointType","square").Vertices;
% 
% 

% for i = 1 : 4
%     hold on
%     plot(mapBoundary(i, [1, 3]), mapBoundary(i, [2, 4]));
% end
% % hold on
% % plot(shrunk_vertices(1:2, 1).', shrunk_vertices(1:2, 2).');
% % hold on
% % plot(shrunk_vertices(2:3, 1).', shrunk_vertices(2:3, 2).');
% % hold on
% % plot(shrunk_vertices(3:4, 1).', shrunk_vertices(3:4, 2).');
% % hold on
% % plot(shrunk_vertices([1, 4], 1).', shrunk_vertices([1,4], 2).');
% 
for i = 1 : length(edgesPath)
    hold on
    plot(edgesPath(i, [1, 3]), edgesPath(i, [2,4]), 'g-', 'LineWidth', 1.5);
end
% hold on
% plotSquareMap(map);
% hold on
% scatter(allWaypoints(:, 1), allWaypoints(:, 2), 'r*', 'LineWidth', 1.5);
