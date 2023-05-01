function [edgeMatrix, edges] = createRoadmap(xv, yv, nodes, factor)
% This function takes in a polygonal environment and returns a roadmap
% covering Q_free
% Inputs:
%           map         A polygonal environemnt
%           xv          processed xv used for inpolygon function
%           yv          processed yv used for inpolygon function
%           nodes       A set of candidate nodes that are in the graph
%           factor      the number of sections needed to divide a line --
%                       for intersection checking
% Outputs:
%           edges       An M by 4 matrix containing the edges
%           edgeMatrix  Each entry records the distance of this edge, the
%                       if 0 -> no edge
%           xv          for inPolygon processing
%           yv          for inPolygon processing
% isVisible is a function that returns 1 if we connect two points and there
% is a colision path.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N is the number of vertices/nodes
[nodeNum, ~] = size(nodes);
edges = []; 
edgeMatrix = zeros(nodeNum);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loop through every node and see if we can construct an edge between
for i = 1 : nodeNum
    currentNode = nodes(i, 1:2);
    for j = 1 : nodeNum
        compareNode = nodes(j, 1:2);
        % There should not be an edge between the itself and itself
        if i ~= j
            visible = isVisible(currentNode, compareNode, xv, yv, factor);
            % If the nodes belong to the same obstacle
            if nodes(i, 3) == nodes(j, 3)
                obstacle = nodes(i, 3); % Get which obstacle we are examining
                
                % This is the point to use to check whether the edge crosses any
                % obstacles
                checkPt = [(currentNode(1) + compareNode(1))/2,...
                    (currentNode(2) + compareNode(2))/2];
           
                [in, on] = inpolygon(checkPt(1), checkPt(2), xv(obstacle, :).', yv(obstacle, :).');
                
                if on == 1 && visible
                    edges = [edges; currentNode, compareNode];
                    edgeMatrix(i, j) = norm(currentNode - compareNode);
                end

            % If the vertices does not belong to the same obstacle
            else
                if visible
                    edges = [edges; currentNode, compareNode];
                    edgeMatrix(i, j) = norm(currentNode - compareNode);
                end
            end
        end
    end
end
