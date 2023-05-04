function G = GjacDiffDrive(x, u)
% GjacDiffDrive: output the jacobian of the dynamics. Returns the G matrix
%
%   INPUTS
%       x            3-by-1 vector of pose: [x; y; theta]
%       u            2-by-1 vector [d, phi]'
%
%   OUTPUTS
%       G            Jacobian matrix partial(g)/partial(x)
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 4
%   Lin, Janna

theta = x(3, 1);
d = u(1, 1);
phi = u(2, 1);
% If phi = 0
if phi == 0
    G = [1, 0, (-d*sin(theta)); 
        0, 1, (d*cos(theta)); 
        0, 0, 1];

else
    G = [1, 0, (-2*(d/phi)*sin(phi/2)*sin(theta + phi/2)); 
        0, 1, (2*(d/phi)*sin(phi/2)*cos(theta + phi/2)); 
        0, 0, 1];
end
end