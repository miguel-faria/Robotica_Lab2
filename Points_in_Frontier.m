function [ frontier ] = Points_in_Frontier( point_leaf, tree )
%Points_in_Frontier - Obtains the points in the frontier of the tree with
%the given leaf

frontier = [];
col = [point_leaf.major_points(1,2) - 1; point_leaf.major_points(2,2)+1];
for lin = point_leaf.major_points(1,1)-1:point_leaf.major_points(2,1)+1
    leaf = Find_Point([lin col(1,1)], tree);
    leaf2 = Find_Point([lin col(2,1)], tree);
    if ~isempty(leaf) && Already_In_Frontier_List(leaf.average_point, frontier) == 0....
            && leaf.type == 0 && ~Same_Average_Position(leaf, point_leaf);
       frontier = [frontier; leaf.average_point]; 
    end
    if ~isempty(leaf2) && Already_In_Frontier_List(leaf2.average_point, frontier) == 0....
            && leaf2.type == 0 && ~Same_Average_Position(leaf2, point_leaf);
       frontier = [frontier; leaf2.average_point]; 
    end
end

lin = [point_leaf.major_points(1,1) - 1; point_leaf.major_points(2,1)+1];
for col = point_leaf.major_points(1,2)-1:point_leaf.major_points(2,2)+1
    leaf = Find_Point([lin(1,1) col], tree);
    leaf2 = Find_Point([lin(2,1) col], tree);
    if ~isempty(leaf) && Already_In_Frontier_List(leaf.average_point, frontier) == 0....
            && leaf.type == 0 && ~Same_Average_Position(leaf, point_leaf);
       frontier = [frontier; leaf.average_point]; 
    end
    if ~isempty(leaf2) && Already_In_Frontier_List(leaf2.average_point, frontier) == 0....
            && leaf2.type == 0 && ~Same_Average_Position(leaf2, point_leaf);
       frontier = [frontier; leaf2.average_point]; 
    end
end

end

function [ output ] = Already_In_Frontier_List(point_coords, list)

    list_size = size(list);
    
    if list_size(1) == 0
        output = 0;
    else
       for i = 1:list_size(1)
           if list(i,1) == point_coords(1,1) && list(i,2) == point_coords(1,2) 
              output = 1;
              return;
           end
       end
       output = 0;
    end
end

function output = Same_Average_Position(leaf1, leaf2)

leaf1_avg_pos = leaf1.average_point;
leaf2_avg_pos = leaf2.average_point;

output = ((leaf1_avg_pos(1,1) == leaf2_avg_pos(1,1)) && (leaf1_avg_pos(1,2) == leaf2_avg_pos(1,2)));
end