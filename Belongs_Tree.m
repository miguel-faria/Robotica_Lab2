function [output] = Belongs_Tree(point_coords, tree)
%Belongs_Tree - Verifies if the given point coordinates are from a point
%covered by the tree

    initial_coords = tree.major_points(1,:);
    end_coords = tree.major_points(2,:);

    output = ((initial_coords(1,1) <= point_coords(1,1) && point_coords(1,1) <= end_coords(1,1)) &&...
                (initial_coords(1,2) <= point_coords(1,2) && point_coords(1,2) <= end_coords(1,2)));

end

