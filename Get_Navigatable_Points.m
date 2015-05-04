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
viable_points = Get_Viable_Navigation_Points(s_tree, []);
n_viable_points = size(viable_points);
nav_points = [];

for i = 1:n_viable_points(1,1)
   current_point.point = viable_points(i,:);
   leaf = Find_Point(current_point.point, s_tree);
   current_point.frontier = Points_in_Frontier(leaf, s_tree);
%    if i > 1
%     plot(viable_points(1:i-1,1),viable_points(1:i-1,2), 'b*');
%    end
%    plot(current_point.point(1,1), current_point.point(1,2), 'r*');
%    if ~isempty(current_point.frontier)
%     plot(current_point.frontier(:,1), current_point.frontier(:,2), 'b*');
%    end
%    pause(0.05);
   nav_points = [nav_points; current_point];
end

end

