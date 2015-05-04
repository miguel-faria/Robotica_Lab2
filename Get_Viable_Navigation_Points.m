function [ points ] = Get_Viable_Navigation_Points( tree, points )
%Get_Viable_Navigation_Points - Obtains the points where the robot can move
%from the given tree obtained from processing the given map.

if tree.type == 0
    points = [points; tree.average_point];
elseif tree.type == 2 && tree.edge_len > 10
    points = Get_Viable_Navigation_Points(tree.descendant1, points);
    points = Get_Viable_Navigation_Points(tree.descendant2, points);
    points = Get_Viable_Navigation_Points(tree.descendant3, points);
    points = Get_Viable_Navigation_Points(tree.descendant4, points);
    points = Get_Viable_Navigation_Points(tree.descendant5, points);
    points = Get_Viable_Navigation_Points(tree.descendant6, points);
    points = Get_Viable_Navigation_Points(tree.descendant7, points);
    points = Get_Viable_Navigation_Points(tree.descendant8, points);
    points = Get_Viable_Navigation_Points(tree.descendant9, points);
end

end