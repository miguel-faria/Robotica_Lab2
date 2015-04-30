function [ search_tree ] = Get_Map_Search_Tree( map )
%Get_Map_Search_Tree - creates a search tree for the given map that is used
%to generate a given trajectory. Note: the map must have heigth equal to
%the width.

map_len = size(map);
if map_len(1,1) ~= map_len(1,2)
    error('The map must be squared, that is, the heigth must be equal to the width');
end

initial_node = Create_Node(map, map_len, [], [1 1; map_len(1,1) map_len(1,2)]);

search_tree = Expand_Node(initial_node);
end

