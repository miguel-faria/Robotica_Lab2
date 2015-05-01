function [ sp_ep_path ] = A_star_search( starting_point, objective_point ,  s_tree, nav_points )
%A_star_search - Does a A star search between the start point (sp) and end
%point (ep) in the given s_tree

start_leaf = Find_Point(starting_point, s_tree);
objective_leaf = Find_Point(objective_point, s_tree);

if ~ismember(start_leaf.average_point, nav_points)
   error('Starting point must be a navigable point');
end

if ~ismember(objective_leaf.average_point, nav_points)
   error('Objective point must be a navigable point');
end

close_list = [];
initial_node = Create_A_star_node(starting_point, [], objective_point);
open_list = initial_node;
open_list_pos = initial_node.position;
objective_reached = 0;

while ~isempty(open_list) && objective_reached == 0

    current_node = open_list(1);
    open_list = open_list(2:end);
    open_list_pos = open_list_pos(2:end,:);
    sp_ep_path = [];
    current_leaf = Find_Point(current_node.position, s_tree);
    
    if Belongs_Tree(objective_point, current_leaf) == 1
        
        objective_reached = 1;
        while ~isempty(current_node)
            sp_ep_path = [current_node.position; sp_ep_path];
            current_node = current_node.dad;
        end
        sp_ep_path = [sp_ep_path; objective_point];
                
    else
        [open_list, open_list_pos, close_list] = Expand_Node(current_node, open_list, open_list_pos, close_list, s_tree, objective_point);        
    end
end
end

function [ node ] = Create_A_star_node(node_pos, dad, objective_pos)

node.position = node_pos;
if isstruct(dad) && ~isempty(dad)
    node.cost = dad.cost + sqrt((node_pos(1,1)-objective_pos(1,1))^2 + (node_pos(1,2)-objective_pos(1,2))^2);
    node.depth = dad.depth + 1;
else
   node.cost = norm(node_pos.*objective_pos);
   node.depth = 0;
end
node.dad = dad;
end

function [open_list_out, open_list_pos, close_list_out] = Expand_Node(node, open_list_in, open_list_pos_in, close_list_in, tree, objective_pos)

open_list_out = open_list_in;
open_list_pos = open_list_pos_in;
close_list_out = close_list_in;
node_pos = node.position;
node_leaf = Find_Point(node_pos, tree);
adjacent_nodes = Points_in_Frontier(node_leaf, tree);
n_adjacent = size(adjacent_nodes);

if ~Already_Closed(node.position, close_list_in)
    for i = 1:n_adjacent(1,1)
        adjacent_node_pos = adjacent_nodes(i,:);
        adjacent_node = Create_A_star_node(adjacent_node_pos, node, objective_pos);
        if ~Already_Closed(adjacent_node_pos, close_list_in)
            [open_list_out, open_list_pos] = Insert_Open_List(adjacent_node, open_list_out, open_list_pos);
        end
    end
    
    close_list_out = [close_list_in; node.position];
end

end

function output = Already_Closed(node_position, close_list)

close_list_size = size(close_list);
output = false;

for i = 1:close_list_size(1,1)
    if node_position(1,1) == close_list(i,1) && node_position(1,2) == close_list(i,2)
        output = true;
        break;
    end
end
end

function [open_list_out, open_list_pos] = Insert_Open_List(node, open_list, open_list_pos_in)

list_size = length(open_list);
open_list_out = open_list;
open_list_pos = open_list_pos_in;

if list_size == 0
    open_list_out = [node];
    open_list_pos = [node.position];
else
    done = 0;
    if open_list(1,1).cost > node.cost
        open_list_out = [node open_list];
        open_list_pos = [node.position; open_list_pos_in];
    else
        for i = 2:list_size
            if open_list(1,i).cost > node.cost
                open_list_out = [open_list(1:i-1) node open_list(i:end)];
                open_list_pos = [open_list_pos_in(1:i-1,:); node.position; open_list_pos_in(i:end,:)];
                done = 1;
                break;
            end
        end

        if done == 0
            open_list_out = [open_list_out node];
            open_list_pos = [open_list_pos_in; node.position];
        end
    end
end
end