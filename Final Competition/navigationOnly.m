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
                   'roadmap', {},...
                   'nodesPath', {};
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
anglesLine = linspace(deg2rad(27), deg2rad(-27), 9);
anglesLine = transpose(anglesLine);

robotRadius = 0.2;

% Wall offset for RoadMap
off = robotRadius + 0.4;

% Real Wall Offset
offReal = 0.05;

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
% EKF Initialization - dataStore and Covariance Matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_rs_rays = 9;
sensorPos = [0,0.8];
[nbeacon, ~] = size(beaconLoc);

% Q and R Matricies
Q = 0.1*eye(2*nbeacon + 9);
Q(:, 1:2*nbeacon) = Q(:, 1:2*nbeacon)/100;
R = 0.01*eye(3);

initSigma = [0.1,0,0;0,0.1,0;0,0,0.1];

% Initalize to the truth pose
[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
dataStore.ekfMu(1,:) = [toc, dataStore.truthPose(end, 2:4)];
dataStore.ekfSigma(1,:) = [toc, reshape(initSigma, [1,9])];
dataStore.deadReck(1,:) = [toc, dataStore.truthPose(1, 2:4)];



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
[newXv, newYv, ~, nodes, realMap] = bufferMap(tempMap, off, off+0.05, offReal, mapVertices);
realMap = [mapBoundary; realMap];
% Get the basic visibility roadmap
[edgeMatrix, edges] = createRoadmap(newXv, newYv, nodes, factor);
[nodeNum, ~] = size(nodes);
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
dataStore.roadmap{1} = [toc reshape(nodes, 1, nodeNum * 3)];
dataStore.nodesPath{1} = [toc reshape(path, 1, pr*pc)];

rmUpdate = 2;
nodeUpdate = 2;
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
    [noRobotCount,dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % EKF - Anonymous Func Pass in integrateOdom as g function
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    g = @(pose, d, phi) integrateOdom(pose, d, phi);
    Gjac = @(x, u) GjacDiffDrive(x, u);
    dataStore.deadReck = [dataStore.deadReck; toc transpose( ...
        g(transpose(dataStore.ekfMu(end, 2:4)), ...
        dataStore.odometry(end, 2), dataStore.odometry(end, 3)))];
    
    % Initialize previous states
    mu_t_bar = transpose(dataStore.ekfMu(end, 2:end));
    sigma_prev = reshape(dataStore.ekfSigma(end, 2:end), 3, 3);
    u_t = transpose(dataStore.odometry(end, 2 : end));
    
    % Get the measurement data from real sense data
    z_t = dataStore.rsdepth(end, 3:end)';

    % Get beacon measurement
    init_t = dataStore.truthPose(end,1);
    if isequal(dataStore.beacon, [])
        tags = [];
    else
        tags = dataStore.beacon(dataStore.beacon(:,1) > init_t, 2:end);
    end
    z_beacon_t = createMeasure(tags, beaconLoc);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % EKF - Getting Localization for MP
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Since the sensor will return 0 if the distance is less than 17.5cm, 
    % we change it to NAN
    [rowNum, colNum] = size(z_t);
    for i = 1 : rowNum
        if z_t(i) < 0.175 || z_t(i) > 10
                z_t(i) = NaN;
        end
    end
    z_t = [z_beacon_t; z_t];

    % Slice all NaN values from map
    map_EKF = realMap(~isnan(realMap(:,1)), :);

    % Pass measurment function and its Jacobian as anonymous functions
    measureJac = @(x) [HBeacont(x, sensorPos, beaconLoc); HjacDepth(x, map_EKF, sensorPos, n_rs_rays)]; 
    h = @(x) [hBeacon(x, sensorPos, beaconLoc); depthPredict(x, map_EKF, sensorPos, anglesLine)];
    
    % Call the EKF function with depth measurement
    [mu_depth, sigma_depth] = EKF(g, h, R, Q, mu_t_bar, sigma_prev, ...
            u_t, z_t, Gjac, measureJac);

    % Store the outputs in dataStore
    dataStore.ekfMu = [dataStore.ekfMu; toc transpose(mu_depth)];
    dataStore.ekfSigma = [dataStore.ekfSigma; toc reshape(sigma_depth, 1, 9)];

    
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % In case the robot hits the wall
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    bumped = dataStore.bump(end, [2, 3, 7]);
    
    if any(bumped)
        SetFwdVelAngVelCreate(Robot, 0, 0);
        TravelDistCreate(Robot, 0.2, -0.5);

        %%%%%%%%%%%%%%%%%
        % Relocalize here
        %%%%%%%%%%%%%%%%%
        



        %%%%%%%%%%%%%%%%%
        % Find a new path
        %%%%%%%%%%%%%%%%%
        start = dataStore.truthPose(end, [2, 3]);
        [path, ~] = findPath(nodes, edgeMatrix, start, allWaypoints(goalIndex, [1, 2]), newXv, newYv, factor);
        gotopt = 1;
        nextPt = path(gotopt,:);
        waypointX = nextPt(1,1);
        waypointY = nextPt(1,2);
        cmdVx = waypointX - currentPoseX;
        cmdVy = waypointY - currentPoseY;
    end
    
    if ~all(isnan(allWaypoints))
        if sqrt((cmdVx)^2+(cmdVy)^2)<=closeEnough
            gotopt = gotopt+1;
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
                    twoNodes = allWaypoints(f, :);
                    [sel, ~] = find(twoNodes == allWaypoints(goalIndex, :));
                    otherPt = twoNodes(unique(sel));

                    allWaypoints(f, :) = nan;

                    currentWall = optWalls(optWallNum, :);  
                    x1 = currentWall(1,1);
                    x2 = currentWall(1,3);
                    y1 = currentWall(1,2);
                    y2 = currentWall(1,4);

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Let the Robot turn to the Wall
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % If the wall is y orientation
                    if abs(x1-x2) <= 0.2
                        % To the right
                        if currentPoseX > min(x1, x2)
                            if currentTheta >= 0
                                TurnCreate(Robot, 0.2, rad2deg(pi-currentTheta));
                            else
                                TurnCreate(Robot, 0.2, -rad2deg((pi+currentTheta)));
                            end
                        % To the left
                        else
                            TurnCreate(Robot, 0.2, -rad2deg(currentTheta));
                        end
                    
                    % If the wall is x orientation
                    else
                        % To the top
                        if currentPoseX > min(y1, y2)
                            if abs(currentTheta) < pi/2
                                TurnCreate(Robot, 0.2, rad2deg(-currentTheta-pi/2));
                            elseif currentTheta > pi/2
                                TurnCreate(Robot, 0.2, rad2deg(3/2*pi - currentTheta));
                            else
                                TurnCreate(Robot, 0.2, rad2deg(- currentTheta-pi/2));
                            end
                        % To the bottom
                        else
                            if abs(currentTheta) < pi/2
                                TurnCreate(Robot, 0.2, rad2deg(-currentTheta+pi/2));
                            elseif currentTheta > pi/2
                                TurnCreate(Robot, 0.2, rad2deg(pi/2-currentTheta));
                            else
                                TurnCreate(Robot, 0.2, rad2deg(cureentTheta- 3/2*pi));
                            end
                        end
                    end
                
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Let robot moves straight until 
                    % it reaches the wall/hit the wall
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    wall_here = 0;
                    
                    hit = 0;
                    prevPose = [currentPoseX, currentPoseY];

                    while norm([dataStore.truthPose(end, [2, 3])] - otherPt) > 0.2 && hit == 0
                        if any(dataStore.bump(end, [2, 3, 7]))
                            SetFwdVelAngVelCreate(Robot, 0, 0);
                            TravelDistCreate(Robot, 0.2, -0.3);
                            wall_here = 1;
                            hit = 1;
                            %%%%%%%%%%%%%%%%%
                            % Relocalize Here
                            %%%%%%%%%%%%%%%%%







                        else
                            SetFwdVelAngVelCreate(Robot, 0.2, 0);
                        end

                        [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
                        if norm(prevPose - dataStore.truthPose(end, [2, 3])) > 0.7
                            hit = 1;
                            SetFwdVelAngVelCreate(Robot, 0, 0);
                        end
                    end

                    SetFwdVelAngVelCreate(Robot, 0, 0);

                    if ~wall_here
                        lastMap(1, optWallNum+n) = 0;
                        dataStore.map = [dataStore.map; toc lastMap];

                        realMap(4*(optWallNum+n)+1 : 4*(optWallNum+n)+4, :) = NaN;

                        % Remove not there opt walls
                        newXv = [newXv(1:(n + optWallNum-1), :); newXv((n+optWallNum+1) : end, :)];
                        newYv = [newYv(1:(n + optWallNum-1), :); newYv((n+optWallNum+1) : end, :)];
                        % Remove not there opt walls' nodes
                        [r, ~] = find(nodes(:, 3) == optWallNum+n);
                        nodes = [nodes(1: (min(r) - 1), :); [nodes((max(r)+1) : end, 1:2), nodes((max(r)+1):end, 3)-1]];
                        [edgeMatrix, edges] = createRoadmap(newXv, newYv, nodes, factor);
                        [nodeNum, ~] = size(nodes);
                        dataStore.roadmap{rmUpdate} = [toc reshape(nodes, 1, nodeNum * 3)];
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
                    [pr, pc] = size(path);
                    dataStore.nodesPath{nodeUpdate} = [toc reshape(path, 1, pr * pc)];
                    nodeUpdate = nodeUpdate + 1;
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

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pause(0.1);
    end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);

end

