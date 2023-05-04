function [dataStore] = EKFTest(Robot, maxTime)
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
sensorPos = [0, 0.08];

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
% tempMap = [map; optWalls];
% 
% optNodes = zeros(2*m, 3);
% 
% for i = 1 : m
%     currWall = optWalls(i, :);
%     midPt = [0.5 * (currWall(1, 1) + currWall(1, 3)), 0.5 * (currWall(1, 2) + currWall(1, 4))];
%     wallLength = norm(currWall(1, [1, 2]) - currWall(1, [3, 4]));
%     orthoX = - (currWall(1, 4) - currWall(1, 2)) / wallLength;
%     orthoY = (currWall(1, 3) - currWall(1, 1)) / wallLength;
%     offPt1 = midPt + (off + 0.05).* [orthoX, orthoY];
%     offPt2 = midPt - (off + 0.05).* [orthoX, orthoY];
%     optNodes(2*i - 1, :) = [offPt1, i]; 
%     optNodes(2*i, :) = [offPt2, i];
% end

% a k × 2 matrix of k waypoints given as [x, y]
waypoints = data.waypoints;
% [k, ~] = size(waypoints);
% % Construct a new set of waypoints where optional wall waypoints can be
% % included; the 3rd column shows if the point is waypoint or wall
% waypoints = horzcat(waypoints, zeros(k, 1));
% 
% % j × 2 matrix of j waypoints given as [x, y]
% ECwaypoints = data.ECwaypoints;
% [j, ~] = size(ECwaypoints);
% ECwaypoints = horzcat(ECwaypoints, -1 * ones(j, 1));
% 
% allWaypoints = [waypoints; ECwaypoints; optNodes];

% an b × 3 matrix where each row gives [tagN um, x, y] for a single beacon
beaconLoc = data.beaconLoc;
[b, ~] = size(beaconLoc);

tic


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Run Particle Filter and Get Locailization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Set velocity back to 0 
SetFwdVelAngVelCreate(Robot, 0, 0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize other dataStore fields
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% We assume all walls are there
dataStore.map = [toc, ones(1, m+n)];

SetFwdVelAngVelCreate(Robot, 0, 0);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize EKF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurement and Dynamics functions, jacobians, Q & R
n_rs_rays = 9;
sensorPos = [0,0.8];
[nbeacon, ~] = size(beaconLoc);
dynamicsJac = @(x,u) GjacDiffDrive(x, u); %pointer to jacobian of the dynamics
measureJac = @(x) [HBeacont(x, sensorPos, beaconLoc); HjacDepth(x, data.map, sensorPos, n_rs_rays)]; 
h = @(x) [hBeacon(x, sensorPos, beaconLoc); depthPredict(x, data.map, sensorPos, linspace(3*pi/20, -3*pi/20, n_rs_rays))];
g = @(x,u) integrateOdom(reshape(x, [3,1]), u(1), u(2));
Q = 0.1*eye(2*nbeacon + 9);
Q(:, 1:2*nbeacon) = Q(:, 1:2*nbeacon)/100;
R = 0.01*eye(3);
initSigma = [0.1,0,0;0,0.1,0;0,0,0.1];

% Initalize to the truth pose
[noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
dataStore.ekfMu(1,:) = [toc, dataStore.truthPose(end, 2:4)];
dataStore.ekfSigma(1,:) = [toc, reshape(initSigma, [1,9])];
dataStore.deadReck(1,:) = [toc, dataStore.truthPose(1, 2:4)];

% Initialize change in angle and change in distance variables
deltaD = 0.0;
deltaA = 0.0;
bump = false;

% Vector of waypoints, epsilon, closeEnough
waypoints = [-2 -2];
epsilon = 0.2;
closeEnough = 0.1;
gotopt = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while toc < maxTime
    
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore] = readStoreSensorData(Robot,noRobotCount,dataStore);
    cmdVx = 0;
    cmdVy = 0;

    % Run EKF Filter
    d = dataStore.odometry(end,2);
    phi = dataStore.odometry(end,3);
    init_t = dataStore.truthPose(end,1);
    
    % Get tags for current time step
    if isequal(dataStore.beacon, [])
        tags = [];
    else
        tags = dataStore.beacon(dataStore.beacon(:,1) > init_t, 2:end);
    end
    z_t = [createMeasure(tags, beaconLoc); dataStore.rsdepth(end,3:11)'];
    %[mu, sigma] = EKFju(dataStore.ekfMu(end,2:end)', [d,phi], reshape(dataStore.ekfSigma(end, 2:end), [3,3]), R, z_t, Q, g, h, dynamicsJac, measureJac);
    [mu, sigma] = EKF(g, h, R, Q,  dataStore.ekfMu(end,2:end)', reshape(dataStore.ekfSigma(end, 2:end), [3,3]), [d phi], z_t, dynamicsJac, measureJac);
    dataStore.ekfMu(end+1, :) = [toc, mu'];
    dataStore.ekfSigma(end+1, :) = [toc, reshape(sigma, [1,9])];
    dataStore.deadReck(end+1, :) = [toc, integrateOdom(dataStore.deadReck(end, 2:end)', d, phi)'];
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        posVector = dataStore.truthPose(end,2:3);
        % If close enough to current waypoint and not at end of waypoints
        if norm(posVector - waypoints(gotopt, :)) <= closeEnough && gotopt < length(waypoints)
            gotopt = gotopt + 1; 
        % If close enough to current waypoint and at end of waypoints
        elseif norm(posVector - waypoints(gotopt, :)) <= closeEnough
            % Goal reached, stop robot
            SetFwdVelAngVelCreate(Robot, 0, 0);
        else
            % Calculate V and w based on waypoint and current position
            cmdVx = waypoints(gotopt,1) - dataStore.truthPose(end,2);
            cmdVy = waypoints(gotopt,2) - dataStore.truthPose(end,3);
            [cmdV, cmdW] = feedbackLin(cmdVx, cmdVy, dataStore.truthPose(end,4), epsilon);
            [cmdV, cmdW] = limitCmds(cmdV, cmdW, 0.2, 0.2);
            SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
        end
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0, 0);

end