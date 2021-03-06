function [ tree ] = Expand_Node( node )
%Expand_Node - expands a node and generates it's descendants, if they exist

tree = node;
if node.type == 2 && node.edge_len > 10
    tree = Generate_Descendants(node);
    tree.descendants = 1;
else
    tree.descendants = 0;
end
end

function [ tree ] = Generate_Descendants(parent_node)
%Generate_Descendants - creates de descendants for a given node

tree = parent_node;
division_len = fix(parent_node.edge_len/3);
parent_start_points = parent_node.major_points(1,:);
xi = parent_start_points(1,2);
yi = parent_start_points(1,1);
parent_end_points = parent_node.major_points(2,:);
xf = parent_end_points(1,2);
yf = parent_end_points(1,1);

descendant1 = Create_Node(parent_node.state(1:division_len, 1:division_len) , division_len, parent_node,...
                            [[yi xi]; [yf xf] - 2*division_len]);
descendant2 = Create_Node(parent_node.state(1:division_len, division_len+1:2*division_len) , division_len, parent_node,...
                            [yi, xi + division_len; [yf xf] - [2*division_len division_len]]);
descendant3 = Create_Node(parent_node.state(1:division_len, 2*division_len+1:parent_node.edge_len) , division_len, parent_node,...
                            [yi, xi + 2*division_len; [yf xf] - [2*division_len 0]]);
descendant4 = Create_Node(parent_node.state(division_len+1:2*division_len, 1:division_len) , division_len, parent_node,...
                            [yi + division_len, xi; [yf xf] - [division_len 2*division_len]]);
descendant5 = Create_Node(parent_node.state(division_len+1:2*division_len, division_len+1:2*division_len) , division_len, parent_node,...
                            [yi + division_len, xi + division_len; [yf xf] - division_len]);
descendant6 = Create_Node(parent_node.state(division_len+1:2*division_len, 2*division_len+1:parent_node.edge_len) , division_len, parent_node,...
                            [yi + division_len, xi + 2*division_len; [yf xf] - [division_len 0]]);
descendant7 = Create_Node(parent_node.state(2*division_len+1:parent_node.edge_len, 1:division_len) , division_len, parent_node,...
                            [yi + 2*division_len, xi; [yf xf] - [0 2*division_len]]);
descendant8 = Create_Node(parent_node.state(2*division_len+1:parent_node.edge_len, division_len+1:2*division_len) , division_len, parent_node,...
                            [yi + 2*division_len, xi + division_len; [yf xf] - [0 division_len]]);
descendant9 = Create_Node(parent_node.state(2*division_len+1:parent_node.edge_len, 2*division_len+1:parent_node.edge_len) , division_len, parent_node,...
                            [yi + 2*division_len, xi + 2*division_len; [yf xf]]);

tree.descendant1 = Expand_Node(descendant1);
tree.descendant2 = Expand_Node(descendant2);
tree.descendant3 = Expand_Node(descendant3);
tree.descendant4 = Expand_Node(descendant4);
tree.descendant5 = Expand_Node(descendant5);
tree.descendant6 = Expand_Node(descendant6);
tree.descendant7 = Expand_Node(descendant7);
tree.descendant8 = Expand_Node(descendant8);
tree.descendant9 = Expand_Node(descendant9);
end