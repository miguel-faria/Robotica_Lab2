function [ point_leaf ] = Find_Point( point_coords, tree )
%Find_Point - Obtain the leaf containing the given point in the tree

point_leaf = [];

if tree.descendants == 0
    if Belongs_Tree(point_coords, tree) == 1
        point_leaf = tree;
    end
else
    if Belongs_Tree(point_coords, tree.descendant1) == 1
        point_leaf = Find_Point(point_coords, tree.descendant1);
        
    elseif Belongs_Tree(point_coords, tree.descendant2) == 1
        point_leaf = Find_Point(point_coords, tree.descendant2);
        
    elseif Belongs_Tree(point_coords, tree.descendant3) == 1
        point_leaf = Find_Point(point_coords, tree.descendant3);
            
    elseif Belongs_Tree(point_coords, tree.descendant4) == 1
        point_leaf = Find_Point(point_coords, tree.descendant4);
            
    elseif Belongs_Tree(point_coords, tree.descendant5) == 1
        point_leaf = Find_Point(point_coords, tree.descendant5);
            
    elseif Belongs_Tree(point_coords, tree.descendant6) == 1
        point_leaf = Find_Point(point_coords, tree.descendant6);
            
    elseif Belongs_Tree(point_coords, tree.descendant7) == 1
        point_leaf = Find_Point(point_coords, tree.descendant7);
            
    elseif Belongs_Tree(point_coords, tree.descendant8) == 1
        point_leaf = Find_Point(point_coords, tree.descendant8);
            
    elseif Belongs_Tree(point_coords, tree.descendant9) == 1
        point_leaf = Find_Point(point_coords, tree.descendant9);
            
    end
end
end