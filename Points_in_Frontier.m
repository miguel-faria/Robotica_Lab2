function [ frontier ] = Points_in_Frontier( point_leaf, tree )
%Points_in_Frontier - Obtains the points in the frontier of the tree with
%the given leaf

frontier = [];
leaf_pos = point_leaf.average_point;
col = [point_leaf.major_points(:,1) - 1; point_leaf.major_points(1,2) + point_leaf.edge_len];
for lin = point_leaf.major_points(1,2)-1:point_leaf.major_points(1,2)+point_leaf.edge_len
    leaf = Find_Point([col(1,1) lin], tree);
    if ~isempty(leaf) && Already_In_Frontier_List(leaf.average_point, frontier) == 0....
            && leaf.type == 0 && leaf.average_point(1,1) ~= leaf_pos(1,1) && leaf.average_point(1,2) ~= leaf_pos(1,2)
       frontier = [frontier; leaf.average_point]; 
    end
end
for lin = point_leaf.major_points(1,2):point_leaf.major_points(1,2)+point_leaf.edge_len
    leaf = Find_Point([col(2,1) lin], tree);
    if ~isempty(leaf) && Already_In_Frontier_List(leaf.average_point, frontier) == 0....
            && leaf.type == 0 && leaf.average_point(1,1) ~= leaf_pos(1,1) && leaf.average_point(1,2) ~= leaf_pos(1,2)
       frontier = [frontier; leaf.average_point]; 
    end
end

lin = [point_leaf.major_points(1,2) - 1; point_leaf.major_points(1,2) + point_leaf.edge_len];
for col = point_leaf.major_points(1,1)-1:point_leaf.major_points(2,1)+point_leaf.edge_len
    leaf = Find_Point([col lin(1,1)], tree);
    if ~isempty(leaf) && Already_In_Frontier_List(leaf.average_point, frontier) == 0....
            && leaf.type == 0 && leaf.average_point(1,1) ~= leaf_pos(1,1) && leaf.average_point(1,2) ~= leaf_pos(1,2)
       frontier = [frontier; leaf.average_point]; 
    end
end
for col = point_leaf.major_points(1,1)-1:point_leaf.major_points(2,1)+point_leaf.edge_len
    leaf = Find_Point([col lin(2,1)], tree);
    if ~isempty(leaf) && Already_In_Frontier_List(leaf.average_point, frontier) == 0....
            && leaf.type == 0 && leaf.average_point(1,1) ~= leaf_pos(1,1) && leaf.average_point(1,2) ~= leaf_pos(1,2)
       frontier = [frontier; leaf.average_point]; 
    end
end

end

function [ output ] = Already_In_Frontier_List(point_coords, list)

    list_size = size(list);
    
    if list_size(1) == 0
        output = 0;
    else
       for i = 1:list_size(1)
           if list(i,:) == point_coords
              output = 1;
              return;
           end
       end
       output = 0;
    end
end