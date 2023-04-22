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
% Initialize needed data:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get the data
data = load('cornerMap.mat');
% an n × 4 matrix consisting of known walls in the environment
map = data.map;
[n, ~] = size(map);
% an m × 4 matrix consisting of optional walls
optWalls = data.optWalls;
[m, ~] = size(optWalls);
% a k × 2 matrix of k waypoints given as [x, y]
waypoints = data.waypoints;
[k, ~] = size(waypoints);
% j × 2 matrix of j waypoints given as [x, y]
ECwaypoints = data.ECwaypoints;
[j, ~] = size(ECwaypoints);
% an b × 3 matrix where each row gives [tagN um, x, y] for a single beacon
beaconLoc = data.beaconLoc;
[b, ~] = size(beacomLoc);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Function Factors and Other Set Up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the sensor range
angles = linspace(deg2rad(27), deg2rad(-27), 9);
angles = transpose(angles);

robotRadius = 0.2;

% Wall offset
off = robotRadius + 0.1;

% Define the sensor origin (from HW2)
sensorPos = [offset_x, offset_y];






SetFwdVelAngVelCreate(Robot, 0, 0);
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
dataStore.map = [toc, ones(1, m)];
% We mark the current waypoint as visited
dataStore.visited.waypoints = [toc, zeros(1, k)];
% Update current waypoint as visited
dataStore.visited.waypoints(1, currWaypoint + 1) = 1;
% Every other point should be marked as unvisited
dataStore.visited.ECwaypoints = [toc, zeros(1, j)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% First Roadmap + Find Path -> 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%













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
    % Set initial forward and angular velocity
    fwdVel = 0.5;
    angVel = -0.3;
    [cmdV, cmdW] =limitCmds(fwdVel, angVel, 0.5, 1.3);
    
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

