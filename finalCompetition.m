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
                   'roadmap', [], ...
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
theta = [];
for i = 1 : 4
    theta(i, 1) = mapVerticesTemp(i, 2)/mapVerticesTemp(i, 1);
end
[B, I] = sort(theta);
mapVertices = zeros(mapVerticesTemp);
for i = 1 : 4
    mapVertices(i, :) = mapVerticesTemp(I(i), :);
end

poly = polyshape(mapVertices(:, 1), mapVertices(:, 2));
mapVertices = polybuffer(poly, -off, "JointType","square").Vertices;

% Buffer the rest of the walls in the map and get new nodes
[newXv, newYv, newMap, nodes] = bufferObstacle(tempMap, off, mapVertices);
% Get the basic visibility roadmap
[edgeMatrix, edges] = createRoadmap(newXv, newYv, nodes, factor);

% Start from the nearest next waypoint
[iterNum, ~] = size(allWaypoints);
path = [];
weightDis = intmax;

% Select the first waypoint to visited -- Least cost
for i = 1 : iterNum
    [pathNodes, d] = findPath(newMap, nodes, edgeMatrix, start, allWaypoints(i, [1, 2]), newXv, newYv, factor);
    if d < weightDis
        goal = allWaypoints(i, [1, 2]);
        weightDis = d;
        path = pathNodes;
    end
end

gotopt = 1;

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
    currentPoseX = dataStore.ekfMu(end, 2);
    currentPoseY = dataStore.ekfMu(end, 3);
    currentTheta = dataStore.ekfMu(end, 4);

    nextPt = allWaypoints(gotopt,:);
    waypointX = nextPt(1,1);
    waypointY = nextPt(1,2);
    cmdVx = waypointX - currentPoseX;
    cmdVy = waypointY - currentPoseY;
    
    if sqrt((cmdVx)^2+(cmdVy)^2)<=closeEnough
        gotopt = gotopt+1;
        if gotopt > length(waypoints)
            cmdVx = 0;
            cmdVy = 0;
        else
            waypointNext = waypoints(gotopt,:);
            waypointX = waypointNext(1,1);
            waypointY = waypointNext(1,2);
            cmdVx = (waypointX - currentPoseX)/alpha;
            cmdVy = (waypointY - currentPoseY)/alpha;
        end
    end

    [fwdVel, angVel] = feedbackLin(cmdVx,cmdVy,currentTheta,epsilon);
    [cmdV, cmdW] = limitCmds(fwdVel, angVel, 0.5, 0.2);
    
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

