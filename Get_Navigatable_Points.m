function [ nav_points, s_tree ] = Get_Navigatable_Points( map )
%Get_Navigatable_Points - Finds the points in the given map where the robot
%can move, that is, the set of points in the white area of the given map
%and returns both the set of points and a search tree whith the points'
%information Note: the map must have heigth equal to the width.
    

map_size = size(map);
if map_size(1,1) ~= map_size(1,2)
    error('The map must be squared, that is, the heigth must be equal to the width');
end

s_tree = Get_Map_Search_Tree(map);
nav_points = Get_Viable_Navigation_Points(s_tree, []);

end

