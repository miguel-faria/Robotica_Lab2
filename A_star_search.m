function [ sp_ep_path ] = A_star_search( starting_point, objective_point ,  s_tree, nav_points, map )
%A_star_search - Does a A star search between the start point (sp) and end
%point (ep) in the given s_tree

start_leaf = Find_Point(starting_point, s_tree);
objective_leaf = Find_Point(objective_point, s_tree);

if ~Is_Navigable(start_leaf.average_point, nav_points)
   error('Starting point must be a navigable point');
end

if ~Is_Navigable(objective_leaf.average_point, nav_points)
   error('Objective point must be a navigable point');
end

close_list = [];
initial_node = Create_A_star_node(starting_point, [], objective_point);
open_list = initial_node;
open_list_pos = initial_node.position;
objective_reached = 0;
sp_ep_path = [];
image(map); hold on;
plot(starting_point(1,2), starting_point(1,1), 'bo');
plot(objective_point(1,2), objective_point(1,1), 'ko');

while ~isempty(open_list) && objective_reached == 0

    current_node = open_list(1);
    open_list = open_list(2:end);
    open_list_pos = open_list_pos(2:end,:);
    current_leaf = Find_Point(current_node.position, s_tree);
    
    if Belongs_Tree(objective_point, current_leaf) == 1
        
        objective_reached = 1;
        current_node = current_node.dad;
        while ~isempty(current_node)
%             plot(current_node.position(1,2), current_node.position(1,1), 'r*');
            dad = current_node.dad;
            if(isempty(dad) || isempty(sp_ep_path) || (sqrt((current_node.position(1,1) - sp_ep_path(1,1))^2 + (current_node.position(1,2) - sp_ep_path(1,2))^2) > 19))
                sp_ep_path = [current_node.position; sp_ep_path];
            end
            current_node = current_node.dad;
        end
        sp_ep_path = [sp_ep_path; objective_point];
%         pause
        
    else
        [open_list, open_list_pos, close_list] = Expand_Node(current_node, open_list, open_list_pos, close_list, s_tree, nav_points, objective_point);  
        plot(close_list(:,2), close_list(:,1), 'b*');
        plot(open_list_pos(:,2), open_list_pos(:,1), 'r*');
    end
end
end

function output = Is_Navigable(point_coords, possible_nav_points)

output = false;
len_nav_points = size(possible_nav_points);
for i = 1:len_nav_points(1,1)
    if (point_coords(1,1) == possible_nav_points(i).point(1,1) && point_coords(1,2) == possible_nav_points(i).point(1,2))
        output = true;
        break;
    end
end

end

function [ node ] = Create_A_star_node(node_pos, dad, objective_pos)

node.position = node_pos;
if isstruct(dad) && ~isempty(dad)
    real_cost = dad.real_cost + sqrt((node_pos(1,1) - dad.position(1,1))^2 + (node_pos(1,2) - dad.position(1,2))^2);
    node.depth = dad.depth + 1;
else
   real_cost = 0; 
   node.depth = 0;
end
node.real_cost = real_cost;
heuristic_cost = sqrt((objective_pos(1,1)-node_pos(1,1))^2 + (objective_pos(1,2)-node_pos(1,2))^2);
node.cost = real_cost + heuristic_cost;
node.dad = dad;
end

function [open_list_out, open_list_pos, close_list_out] = Expand_Node(node, open_list_in, open_list_pos_in, close_list_in, tree, nav_points, objective_pos)

open_list_out = open_list_in;
open_list_pos = open_list_pos_in;
close_list_out = close_list_in;
node_leaf = Find_Point(node.position, tree);
point = Find_Nav_Point(node_leaf.average_point, nav_points);
if ~isempty(point)
    frontier_points = point.frontier;
   if isempty(frontier_points)
       node.cost = realmax;
   else
    n_adjacent = size(frontier_points);
    if ~Already_Closed(node.position, close_list_in)
        for i = 1:n_adjacent(1,1)
            adjacent_node_pos = frontier_points(i,:);
            adjacent_node = Create_A_star_node(adjacent_node_pos, node, objective_pos);
            if ~Already_Closed(adjacent_node_pos, close_list_in)
                [open_list_out, open_list_pos] = Insert_Open_List(adjacent_node, open_list_out, open_list_pos);
            end
        end

    end
   end
   close_list_out = [close_list_in; node.position];
end


end

function [point] = Find_Nav_Point(point_coords, nav_points)

point = [];
len_nav_points = size(nav_points);

for i = 1:len_nav_points(1,1)
    if(point_coords(1,1) == nav_points(i).point(1,1) && point_coords(1,2) == nav_points(i).point(1,2))
        point = nav_points(i);
        break;
    end
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