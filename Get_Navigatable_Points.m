function [ nav_points, s_tree ] = Get_Navigatable_Points( map )
%Get_Navigatable_Points - Finds the points in the given map where the robot
%can move, that is, the set of points in the white area of the given map
%and returns both the set of points and a search tree whith the points'
%information
    
%s_tree = Get_Map_Search_Tree(map);
s_tree = [];
map_size = size(map);
index = 1;

for i = 1:map_size(1,1)
    for j = 1:map_size(1,2)
        if map(i,j) == 255
            nav_points(index,:) = [i j];
            index = index + 1;
        end
    end
end

end

