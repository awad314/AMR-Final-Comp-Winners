function[exp_pos] = hBeacon(pose, sensorPos, beacon)
% hBeacon: gives the expected beacon measurement based on the robot's pose,
% position of sensor, and locations of beacons. 
%
%   INPUTS
%       pose              1x3 vector of x,y,theta
%       sensorPos         2x1 vector of sensor position in robot frame
%       beacon            3xn matrix of beacon positions and labels
%
%   OUTPUTS
%       exp_pos           2*nx1 vector of expected beacon measurments

exp_pos = zeros(2*length(beacon),1);

counter = 1;
for i = 1: length(beacon)
    p = global2robot(pose, beacon(i, 1:2)) - sensorPos;
    exp_pos(counter) = p(1);
    exp_pos(counter + 1) = p(2);
    counter = counter + 2;
end

end

function[xyR] = global2robot(pose,xyG)
% GLOBAL2ROBOT: transform a 2D point in global coordinates into robot
% coordinates (assumes planar world).
% 
%   XYR = GLOBAL2ROBOT(POSE,XYG) returns the 2D point in robot coordinates
%   corresponding to a 2D point in global coordinates.
% 
%   INPUTS
%       pose    robot's current pose [x y theta]  (1-by-3)
%       xyG     2D point in global coordinates (1-by-2)
% 
%   OUTPUTS
%       xyR     2D point in robot coordinates (1-by-2)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Kochersperger, Julia

% Angle between reference frames
theta = pose(3);

% Transformation matrix
tBI = [cos(theta), -sin(theta), pose(1); 
        sin(theta), cos(theta), pose(2); 
        0, 0, 1]^-1;

% Calcular
xyR = tBI*[xyG, 1].';
xyR = xyR(1:2).';

end
