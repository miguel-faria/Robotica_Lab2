function [ node ] = Create_Node( state_map, state_map_len, dad, major_points )
%Create_Node - Creates a tree node with the following information:
%       - state: the map portion that the node stores
%       - dad: the parent node
%       - edge_len: the edge length for the map
%       - major_points: the upper lef corner and lower right corner of the
%       map
%       - average_point: the point in the middle of the stored map portion
%       - type: the type of the map portion stored (See Type_Map_Matrix
%       function)

half_node_len = fix(state_map_len(1,1)/2);

node.state = state_map;
node.dad = dad;
node.edge_len = state_map_len(1,1);
node.major_points = major_points;
node.average_point = [node.major_points(1,1) + half_node_len...
                       node.major_points(1,2) + half_node_len];
node.type = Type_Map_Matrix(state_map);

end

