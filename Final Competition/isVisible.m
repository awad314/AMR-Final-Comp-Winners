function visible = isVisible(currentNode, compareNode, xv, yv, sampNum)
% This helpfer function checks if the prospective edge is actually visible
% Inputs:
%           currentNode     [x, y]
%           compareNode     [x, y]
%           xv              k by M vector of X coordinates
%           yv              k by M vector of Y coordinates
%           sampNum         The number of samples along the line
% Output:
%           visible         either 1 or 0

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize visible
visible = 1;

% Get the number of nodes
[obstacleNum, ~] = size(xv);

distance = norm(-currentNode + compareNode);
sFactor = (-currentNode + compareNode) ./ distance;

%create vector of points along line of
x = currentNode(1) + sFactor(1) .* linspace(0, distance, sampNum);
y = currentNode(2) + sFactor(2) .* linspace(0, distance, sampNum);

for i = 1 : obstacleNum

    list = [];
    for j = 1 : sampNum
        in = inpolygon(x(1, j), y(1, j), xv(i, :), yv(i, :));
        list = [list in];
    end

    count = sum(list(2: (end-1)));

    if count > 0
        visible = 0;
    end
    
end

end