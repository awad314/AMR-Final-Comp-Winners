function z = createMeasure(tags, beacon)
% Inputs: 
%       tags       description of tags seen by the robot
%       beacon     matrix describing locations of beacons in map, of the
%                  form [id1, x1, y1; id2, x2, y2]
%
%
% Outputs:
%       z           2*numTags x 1 vector

% Initialize as all nans
z = NaN(length(beacon), 2);

% Update measurment for each beacon seen
s = size(tags);
for i = 1:s(1)
    idx = find(beacon(:,1) == tags(i, 2));
    z(idx, :) = tags(i, 3:4);
end

% Get into correct format
z = reshape(z', [2*length(beacon), 1]);
end