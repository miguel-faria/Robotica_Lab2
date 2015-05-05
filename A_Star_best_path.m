function [ path_points ] = A_Star_best_path( map, nav_points, s_tree ,waypoints )
%A_Star_best_path - A star search for the best path in a given map that
%passes through the given waypoints. To note that the waypoints must be at
%least 2.

path_temp = [];
n_points = size(waypoints);

if n_points(1,1) < 2
    error('There must be at least two waypoints: one starting point and one destination point.');
end

for i = 1:n_points(1,1)
    
    if i == n_points(1,1)
        break;
    end
    
    start_point = [waypoints(i,2), waypoints(i,1)];
    end_point = [waypoints(i+1,2), waypoints(i+1,1)];
    
    if isempty(path_temp)
        path_temp = [path_temp; A_star_search(start_point, end_point, s_tree, nav_points, map)];
    else
        path_temp = [path_temp(1:end-1,:); A_star_search(start_point, end_point, s_tree, nav_points, map)];
    end
end

image(map); hold on;
if ~isempty(path_temp)
    path_points = [path_temp(:,2) path_temp(:,1)];
    plot(waypoints(:,1), waypoints(:,2), 'ko');
    plot(path_points(:,1), path_points(:,2), 'r*');
    hold off;
else
    path_points = path_temp;
end
end