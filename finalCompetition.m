function [dataStore] = finalCompetition(Robot, maxTime, offset_x, offset_y)
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
    maxTime = 60;
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
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'logodds', [], ...
                   'map', [], ...
                   'particles', [],...
                   'roadmap', {}, ...
                   'visited', []);

% ekfMu: [toc x y theta]
% ekfSigma: [toc reshape(Sigma, 1, 9]
% logodds: [toc reshape]
% map: [toc (Indicator matrix)]
%       Indicator matrix: for every index i, put 1 or 0 representing if
%       optWalls(i, :) is in optional world
% particles:
%   x       [toc x1, x2, x3, ...]
%   y       [toc y1, y2, t3, ...]
%   theta   [toc theta1, theta2, ...]
%   weights [toc w1, w2, w3, ...]
% roadmap:
%   vertices.x [toc x1, x2, x3, ...]
%   vertices.y [toc y1, y2, y3, ...]
%   indicator   N by N
% visited: 
%   normal      [toc (Indicator vector)]
%   EC          [toc (Indicator vector)]


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
off = robotRadius + 0.15;

% Define the sensor origin (from HW2)
sensorPos = [offset_x, offset_y];

alpha = 1;
epsilon = 0.2;

% Number of subsections to take on one line
factor = 50;

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Opt Wall Nodes Construction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The third row is the opt wall index
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% allWaypoints Construction
% k   rows    [waypoint, 0]
% j   rows    [ECWaypoint, -1]
% 2*m rows    [optWall, i]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
allWaypoints = [waypoints; ECwaypoints; optNodes];

% an b × 3 matrix where each row gives [tagN um, x, y] for a single beacon
beaconLoc = data.beaconLoc;
[b, ~] = size(beaconLoc);

tic


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run Particle Filter and Get Locailization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);

% Create two fields in dataStore.particles: three K by 1 vectors storing the
% pose (x, y theta) of each particle respectively, and a K by 1 matrix 
% storing the weights of each particle
dataStore.particles.x = [];
dataStore.particles.y = [];
dataStore.particles.theta = [];
dataStore.particles.weights = [];



% The index of the current waypoint should be identified and returned as
% currWaypoint

% Return current loc as start





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EKF Initialization - dataStore and Covariance Matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize deadReck and ekfMu to the true initial pose - TO BE UPDATED
dataStore.deadReck = [toc, dataStore.truthPose(end, 2 : 4)];
dataStore.ekfMu = [toc, dataStore.truthPose(end, 2 : 4)];
dataStore.ekfSigma = [toc, 2, 0, 0, 0, 2, 0, 0, 0, 0.1];












%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize other dataStore fields
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We assume all walls are there
dataStore.map = [toc, ones(1, m+n)];
% We mark the current waypoint as visited
dataStore.visited.waypoints = [toc, zeros(1, k)];
% Update current waypoint as visited
dataStore.visited.waypoints(1, currWaypoint + 1) = 1;
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
hold on
for i = 1 : pr-1
    plot([path(i, 1), path(i+1, 1)], [path(i,2), path(i+1, 2)], 'b-');
end
hold on
scatter(goal(1),goal(2), 'ro', 'LineWidth', 1.5);
hold on
scatter(start(1), start(2), 'k*', 'LineWidth', 1.5);

SetFwdVelAngVelCreate(Robot, 0, 0);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % EKF - Anonymous Func Pass in integrateOdom as g function
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    g = @(pose, d, phi) integrateOdom(pose, d, phi);
    Gjac = @(x, u) GjacDiffDrive(x, u);
    dataStore.deadReck = [dataStore.deadReck; toc transpose( ...
        g(transpose(dataStore.truthPose(end, 2:4)), ...
        dataStore.odometry(end, 2), dataStore.odometry(end, 3)))];
    
    % Initialize previous states
    mu_t_bar = transpose(dataStore.ekfMu(end, 2:end));
    sigma_prev = reshape(dataStore.ekfSigma(end, 2:end), 3, 3);
    u_t = transpose(dataStore.odometry(end, 2 : end));
    
    % Get the measurement data from real sense data and process NaN data
    z_t = transpose(dataStore.rsdepth(end, 3 : end));

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

        h_depth = @(mu_t_bar) depthPredict(mu_t_bar, map, sensor_pos, angles);
        Hjac_depth = @(mu_t_bar) HjacDepth(mu_t_bar, map, sensor_pos, 9);
        
        % Call the EKF function with depth measurement
        [mu_depth, sigma_depth] = EKF(g, h_depth, R, Q, mu_t_bar, sigma_prev, ...
            u_t, z_t, Gjac, Hjac_depth);
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
                allWaypoints(f, :) = nan;
                
                %%%% Find Angle Needed to Turn to Face Wall %%%
                
                current_wall = optWalls(optWallNum);  
                x1 = current_wall(1,1);
                x2 = current_wall(1,3);
                y1 = current_wall(1,2);
                y2 = current_wall(1,4);
                
                theta = (180/pi)*currentTheta;
                m = (y2-y1)/(x2-x1);
                alpha = atand(m);
                turn = alpha-theta+90;  
                TurnCreate(Robot,0.2,turn);
                if x1==x2
                    [val,x,y] = intersectPoint(x1,y1,x2,y2,cPose(1),cPose(2),cPose(1)+10,cPose(2));
                    if val==1
                      continue
                    else
                      turn=180;
                     TurnCreate(Robot,0.2,turn)
                    end
                 elseif y1==y2
                  [val,x,y] = intersectPoint(x1,y1,x2,y2,cPose(1),cPose(2),cPose(1),cPose(2)+10);
                  if val==1
                    continue
                  else
                     turn=180;
                     TurnCreate(Robot,0.2,turn)   
                  end
                end
                TravelDistCreate(Robot,0.2,0.2);
                bumped = dataStore.rsdepth(end,2:7);
                if any(any(bumped))
                     TravelDistCreate(Robot,-0.2,0.2);
                     wall_here = 1;
                     disp("WALL HERE")
                else
                    wall_here = 0;
                    disp("WALL NOT HERE")
                end                   

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
    
    elseif cmdVx == 0 && cmdVy == 0

    
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
       
    end
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pause(0.1);
    end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);

end