function [dataStore, roadmap] = navigationOnly(Robot, maxTime)
% This function drive the robot in different directions while reacting to
% the bump sensor and can switch between EKF and PF
%   dataStore = motionControl(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
%       offset_x    camera offsets X coordinates
%       offset_y    camera offsets Y coordinates
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.

% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 300;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort = Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialiazation of DataStore
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize datalog struct
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [],...
                   'map', [], ...
                   'visited', []),...
                   'roadmap', {};
% map: [toc (Indicator matrix)]
%       Indicator matrix: for every index i, put 1 or 0 representing if
%       optWalls(i, :) is in optional world
% roadmap:
%   vertices.x [toc x1, x2, x3, ...]
%   vertices.y [toc y1, y2, y3, ...]
%   indicator   N by N
% visited: 
%   normal      [toc (Indicator vector)]
%   EC          [toc (Indicator vector)]
tic

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Function Factors and Other Set Up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the sensor range
angles = linspace(deg2rad(27), deg2rad(-27), 9);
angles = transpose(angles);

robotRadius = 0.2;

% Wall offset
off = robotRadius + 0.35;

alpha = 1;
epsilon = 0.2;

% Number of subsections to take on one line
factor = 50;
closeEnough = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize needed data:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
start = dataStore.truthPose(1, 2:3);
tempDis = Inf;
for i = 1 : k
    toDis = norm(start - waypoints(i, [1, 2]));
    if toDis < tempDis
        currWaypoint = i;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize other dataStore fields
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We assume all walls are there
dataStore.map = [toc, ones(1, n+m)];
% We mark the current waypoint as visited
dataStore.visited.waypoints = [toc, zeros(1, k)];
% Update current waypoint as visited
dataStore.visited.waypoints(1, currWaypoint + 1) = 1;
% BeepRoombaCreate(Robot);
% Every other point should be marked as unvisited
dataStore.visited.ECwaypoints = [toc, zeros(1, j)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First Roadmap + Find Path -> 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Shrink the map boundary inward by fac
mapVerticesTemp = mapBoundary(:, [1, 2]);
centroid = mean(mapVerticesTemp);
translated = mapVerticesTemp - centroid;
angles = atan2(translated(:,2), translated(:,1));
[~, sorted] = sort(angles);
mapVertices = mapVerticesTemp(sorted, :);

poly = polyshape(mapVertices(:, 1), mapVertices(:, 2));
mapVertices = polybuffer(poly, -off, "JointType","square").Vertices;

% Buffer the rest of the walls in the map and get new nodes
[newXv, newYv, ~, nodes] = bufferMap(tempMap, off, off+0.05, mapVertices);
% Get the basic visibility roadmap
[edgeMatrix, edges] = createRoadmap(newXv, newYv, nodes, factor);

% Start from the nearest next waypoint
[iterNum, ~] = size(allWaypoints);
path = [];
weightDis = intmax;

% Select the first waypoint to visited -- Least cost
for i = 1 : iterNum
    [pathNodes, d] = findPath(nodes, edgeMatrix, start, allWaypoints(i, [1, 2]), newXv, newYv, factor);
    if d < weightDis
        goalIndex = i;
        goal = allWaypoints(i, [1, 2]);
        weightDis = d;
        path = pathNodes;
    end
end

[pr, pc] = size(path);
dataStore.roadmap{1} = [toc reshape(path, 1, pr*pc)];
rmUpdate = 2;
gotopt = 1;

[er, ~] = size(edges);

SetFwdVelAngVelCreate(Robot, 0, 0);
figure
for i = 1 : er
    hold on
    plot(edges(i, [1,3]), edges(i, [2, 4]), 'g-');
end
hold on
plotSquareMap(map);
hold on
for i = 1 : pr-1
    plot([path(i, 1), path(i+1, 1)], [path(i,2), path(i+1, 2)], 'b-');
end
hold on
scatter(goal(1),goal(2), 'ro', 'LineWidth', 1.5);
hold on
scatter(start(1), start(2), 'k*', 'LineWidth', 1.5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % CONTROL FUNCTION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    currentPoseX = dataStore.truthPose(end, 2);
    currentPoseY = dataStore.truthPose(end, 3);
    currentTheta = dataStore.truthPose(end, 4);

    nextPt = path(gotopt,:);
    waypointX = nextPt(1,1);
    waypointY = nextPt(1,2);
    cmdVx = waypointX - currentPoseX;
    cmdVy = waypointY - currentPoseY;
    
    if ~all(isnan(allWaypoints))
        if sqrt((cmdVx)^2+(cmdVy)^2)<=closeEnough
            gotopt = gotopt+1
            if gotopt <= length(path)
                hold on
                plot([path(gotopt-1, 1), path(gotopt, 1)], [path(gotopt-1, 2), path(gotopt, 2)]);
            else
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Step 1: Stop when the waypoint is reached
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                SetFwdVelAngVelCreate(Robot, 0, 0);
                cmdVx = 0;
                cmdVy = 0;
            
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Step 2: Identify what the waypoint belongs to
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % If normal waypoint
                if goalIndex <= k
                    lastWaypoints = dataStore.visited.waypoints(end, 2 : end);
                    lastWaypoints(1, goalIndex) = 1;
                    dataStore.visited.waypoints = [dataStore.visited.waypoints; toc lastWaypoints];
                    % BeepRoombaCreate(Robot);
                    allWaypoints(goalIndex, :) = nan;
    
                % If extraw waypoint
                elseif goalIndex <= k+j
                    lastECWaypoints = dataStore.visited.ECwaypoints(end, 2 : end);
                    lastECWaypoints(1, goalIndex - k) = 1;
                    dataStore.visited.ECwaypoints = [dataStore.visited.ECwaypoints; toc lastECWaypoints];
                    % BeepRoombaCreate(Robot);
                    allWaypoints(goalIndex, :) = nan;
            
                % If extra wall node indices
                else
                    lastMap = dataStore.map(end, 2 : end);
                    optWallNum = ceil((goalIndex - k - j)/2);
                    [f, ~] = find(allWaypoints(:, 3)==optWallNum);
                    allWaypoints(f, :) = nan;

                    currentWall = optWalls(optWallNum, :);  
                    x1 = currentWall(1,1);
                    x2 = currentWall(1,3);
                    y1 = currentWall(1,2);
                    y2 = currentWall(1,4);
                
                    theta = (180/pi) * currentTheta;
                    m = (y2-y1)/(x2-x1);
                    lineAng = atand(m);
                    if currentPoseY>min(y1,y2)
                        turn = lineAng-theta-90
                    elseif currentPoseX>min(x1,x2)
                        turn = lineAng-theta-90
                    end
                        
                    turn = lineAng-theta+90;  
                    TurnCreate(Robot, 0.2, turn);
                
                    if abs(x1-x2) <= 0.2
                        [val,x,y] = intersectPoint(x1,y1,x2,y2, ...
                            currentPoseX,currentPoseY,currentPoseX+10,currentPoseY);
                        if val==0
                            turn=180;
                            TurnCreate(Robot,0.2,turn)
                        end
                    
                    elseif abs(y1-y2) <= 0.2
                        [val,x,y] = intersectPoint(x1,y1,x2,y2, ...
                            currentPoseX,currentPoseY,currentPoseX,currentPoseY+10);
                        if val==0
                            turn=180;
                            TurnCreate(Robot,0.2,turn)   
                        end
                    end
                
                    midWall = 1/2 * [x2 + x1, y2 + y1];
                    wall_here = 0;
                    
                    hit = 0;
                    while norm([dataStore.truthPose(end, [2, 3])] - midWall)> 0 && hit == 0
                        if any(dataStore.bump(end, [2, 3, 7]))
                            SetFwdVelAngVelCreate(Robot, 0, 0);
                            TravelDistCreate(Robot, 0.2, -0.3);
                            wall_here = 1;
                            hit = 1;
                        else
                            SetFwdVelAngVelCreate(Robot, 0.2, 0);
                        end
                        [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
                    end                   
                    SetFwdVelAngVelCreate(Robot, 0, 0);

                    if ~wall_here
                        lastMap(1, optWallNum+n) = 0;
                        dataStore.map = [dataStore.map; toc lastMap];

                        % Remove not there opt walls
                        newXv = [newXv(1:(n + optWallNum-1), :); newXv((n+optWallNum+1) : end, :)];
                        newYv = [newYv(1:(n + optWallNum-1), :); newYv((n+optWallNum+1) : end, :)];
                        % Remove not there opt walls' nodes
                        [r, ~] = find(nodes(:, 3) == optWallNum+n);
                        nodes = [nodes(1: (min(r) - 1), :); [nodes((max(r)+1) : end, 1:2), nodes((max(r)+1):end, 3)-1]];
                        [edgeMatrix, edges] = createRoadmap(newXv, newYv, nodes, factor);
                        [pr, pc] = size(path);
                        dataStore.roadmap{rmUpdate} = [toc reshape(path, 1, pr*pc)];
                        rmUpdate = rmUpdate + 1;
                    end
                end

                if ~all(isnan(allWaypoints))
                    weightDis = Inf;
                    start = dataStore.truthPose(end, [2,3]);
                    for i = 1 : iterNum
                    [pathNodes, d] = findPath(nodes, edgeMatrix, start, allWaypoints(i, [1, 2]), newXv, newYv, factor);
                        if d < weightDis
                            goalIndex = i;
                            goal = allWaypoints(i, [1, 2]);
                            weightDis = d;
                            path = pathNodes;
                        end
                    end
                gotopt = 1;
                figure
                for i = 1 : length(edges)
                    hold on
                    plot(edges(i, [1,3]), edges(i, [2, 4]), 'g-');
                end
                hold on
                scatter(goal(1), goal(2), 'ro', 'LineWidth', 2);
                hold on
                scatter(start(1), start(2), 'k*', 'LineWidth', 1.5);
                hold on
                plotSquareMap(map);
                else
                    break
                end
            end
        else
            waypointNext = path(gotopt,:);
            waypointX = waypointNext(1,1);
            waypointY = waypointNext(1,2);
            cmdVx = (waypointX - currentPoseX)/alpha;
            cmdVy = (waypointY - currentPoseY)/alpha;
        end
    else
        SetFwdVelAngVelCreate(Robot, 0, o);
    end

    [fwdVel, angVel] = feedbackLin(cmdVx,cmdVy,currentTheta,epsilon);
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, 0.2, 0.2); 
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
   
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
       
    end
    

    dataStore.truthPose(end, :);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pause(0.1);
    end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);

end

