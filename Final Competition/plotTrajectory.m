function plotTrajectory(map, goal, c_att, c_rep, Q)
% This function plots the trajectory of the robot
% Inputs:
%       map         map of the environment defined by circles. First row is 
%                   the map boundary.
%                   k x 3 matrix [x_center y_center radius]
%       goal        goal point
%                   1 x 2 array [x_goal y_goal]
%       c_att       scaling factor for the atractive force (potential fun).
%       c_rep       scaling factor for the repulsive force (potential fun).
%       Q           influence range of the obstacles (real number)

figure

reach = 0;
robotPose = load('traPo.mat').dataStore.truthPose(:, [2, 3]);
init = robotPose(1, :);
% 
% x = robotPose(1);
% y = robotPose(2);

[a, h] = plotCircleWorld(map);
hold on
q = plot(init(1), init(2), 'r*');
d = plot(goal(1), goal(2), 'g*');

p = plot(robotPose(:, 1).', robotPose(:, 2).', 'b-', 'LineWidth', 2);

% while reach~=1
%     [U, U_grad] = potentialPoint(map, goal, c_att, c_rep, Q, robotPose);
%     robotPose(1) = robotPose(1) - U_grad(1, 1)/100;
%     robotPose(2) = robotPose(2) - U_grad(1, 2)/100;
% 
%     p = plot([x, robotPose(1)], [y, robotPose(2)], 'b-', 'LineWidth', 2);
% 
%     if norm(robotPose - goal) <= 0.2
%         reach = 1;
%     end
% 
%     x = robotPose(1);
%     y = robotPose(2);
% end
% Plot the sphere world
hold on

m = plotSquareMap(load('labBoxMap_wall_orange.txt'));
legend([a, h, q, d, p, m], 'Map Boundary', 'Obstacles', 'Start', 'Goal', 'Trajectory', 'Simulator Wall');

