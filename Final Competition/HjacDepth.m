function H = HjacDepth(x, map, sensor_pos, K)
% HjacDepth: output the jacobian of the depth measurement. Returns the H matrix
%
%   INPUTS
%       x            3-by-1 vector of pose
%       map          of environment, n x [x1 y1 x2 y2] walls
%       sensor_pos   sensor position in the body frame [1x2]
%       K            number of measurements (rays) the sensor gets between 27 to -27 
%
%   OUTPUTS
%       H            Kx3 jacobian matrix
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Lin, Janna

angles = linspace(deg2rad(27), deg2rad(-27), K);
dif = 10^(-6);
angles = transpose(angles);

% Initialize the initial H matrix
H = ones(K, 3);

% Initial depth predict
h_prime = depthPredict(x, map, sensor_pos, angles);

for i = 1 : 3
    x_dif = x;
    x_dif(i, 1) = x_dif(i, 1) + dif;

    H(:, i) = (depthPredict(x_dif, map, sensor_pos, angles) - h_prime)/dif;
end

end