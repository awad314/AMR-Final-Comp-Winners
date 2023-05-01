function [selNodes, d] = findPath(nodes, edgeMatrix, start, goal, xv, yv, factor)
% This function takes in a polygonal environment, a roadmap, an initial and
% goal point to return the shortest path connecting the initial and goal
% points
% Inputs:
%           nodes       The nodes of this roadMap
%           edgeMatrix  N by N matrix
%           start       Initial pose
%           goal        Final Pose -Goal may be nan because we already
%                       visit the waypoint
% Outputs:
%           selNodes    The nodes of the path in sequence
%           d           The cost of the path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update the roadMap with start and goal added
[nodeNum, ~] = size(nodes);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Process s and t arrays without the goal and the start point
% Let nodeNum + 1 denote the index of the start, and nodeNum + 2 denote the
% index of the goal
s = []; 
t = [];
weights = [];

if ~isnan(goal)
    str2Goal = isVisible(start, goal, xv, yv, factor);
    if str2Goal
        s = [s, nodeNum + 1];
        t = [t, nodeNum + 2];
        weights = [weights, norm(start - goal)];
    end

    for i = 1 : nodeNum
        for j = i : nodeNum
            if edgeMatrix(i, j) > 0
                s = [s, i];
                t = [t, j];
                weights = [weights, edgeMatrix(i, j)];
            end
        end

        currNode = nodes(i, [1, 2]);

        visibleStr = isVisible(start, currNode, xv, yv, factor);
        visibleGoal = isVisible(goal, currNode, xv, yv, factor);

        if visibleStr
            s = [s, nodeNum + 1];
            t = [t, i];
            weights = [weights, norm(currNode - start)];
        end

        if visibleGoal
            s = [s, i];
            t = [t, nodeNum + 2];
            weights = [weights, norm(currNode - goal)];
        end

    end

    % Update nodes
    nodes = [nodes; [start 0]; [goal 0]];

    % Construct the graph
    G = graph(s, t, weights);

    if any([all([any(ismember(s, nodeNum + 1)), any(ismember(t, nodeNum + 2))]), all([any(ismember(s, nodeNum + 2)), any(ismember(t, nodeNum + 1))])])
        % Run the shortestpath function
        [path, d] = shortestpath(G, (nodeNum + 1), (nodeNum + 2));
    else
        path = [];
        d = Inf;
    end
    selNodes = zeros(length(path), 2);

    for i = 1 : length(path)
        selNodes(i, :) = nodes(path(i), [1, 2]);
    end
else
    selNodes = nan;
    d = Inf;
end


end