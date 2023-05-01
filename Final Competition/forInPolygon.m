function [obxv, obyv] = forInPolygon(xv, yv)
% This function returns two vectors that only contain effective vertices.
% That being said, it eliminates the points [0, 0]
% Inputs;
%           xv      1 by k vector of X coordinate
%           yv      1 by k vector of Y coordinate
% Outputs:
%           obxv    1 by M vector of X coordinate, s.t. M <= k
%           obyv    1 by M vector of Y coordinate, s.t. M <= k

obxv = [];
obyv = [];

for i = 1 : length(xv)
    if~(xv(i) == 0 && yv(i) == 0)
        obxv = [obxv, xv(i)];
        obyv = [obyv, yv(i)];
    end
end
end