function [finalPose] = integrateOdom(initPose,d,phi)
% integrateOdom: Calculate the robot pose in the initial frame based on the
% odometry
% 
% [finalPose] = integrateOdom(initPose,dis,phi) returns a 3-by-N matrix of the
% robot pose in the initial frame, consisting of x, y, and theta.

%   INPUTS
%       initPose    robot's initial pose [x y theta]  (3-by-1)
%       d     distance vectors returned by DistanceSensorRoomba (1-by-N)
%       phi     angle vectors returned by AngleSensorRoomba (1-by-N)

% 
%   OUTPUTS
%       finalPose     The final pose of the robot in the initial frame
%       (3-by-N)

%   Cornell University
%   MAE 4180/5180 CS 3758: Autonomous Mobile Robots
%   Homework #2
currentX = initPose(1,1);
currentY = initPose(2,1);
currentTheta = initPose(3,1);

[x, dataNum] = size(d);
loopTrack = 1;
finalPose = zeros(3, dataNum);

sz = size(d);
cord = zeros(sz);
for i = 1 : dataNum
    if phi(i) == 0
        cord(i) = d(i);
    else
        cord(i) = 2 * d(i)/phi(i) * sin(phi(i)/2);
    end
end

while loopTrack <= dataNum
    currentPhi = phi(1,loopTrack);
    distance = cord(1, loopTrack);
    
    finalPose(1, loopTrack) = currentX + distance*cos(currentTheta+currentPhi/2);
    finalPose(2, loopTrack) = currentY + distance*sin(currentTheta+currentPhi/2);
    
    currentTheta = currentPhi + currentTheta;
    finalPose(3, loopTrack) = currentTheta;
    
    currentX = finalPose(1, loopTrack);
    currentY = finalPose(2, loopTrack);
    

    loopTrack = loopTrack+1;
end