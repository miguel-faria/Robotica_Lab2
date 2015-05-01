function [ path ] = A_Star_best_path( map, nav_points, s_tree ,waypoints )
%A_Star_best_path - A star search for the best path in a given map that
%passes through the given waypoints. To note that the waypoints must be at
%least 2.

path = [];
n_points = size(waypoints);

if n_points(1,1) < 2
    error('There must be at least two waypoints: one starting point and one destination point.');
end

for i = 1:n_points(1,1)
    
    if i == n_points(1,1)
        break;
    end
    
    start_point = waypoints(i,:);
    end_point = waypoints(i+1,:);
    
    path = [path; A_star_search(start_point, end_point, s_tree, nav_points)];
end

image(map); hold on;
plot(waypoints(:,1), waypoints(:,2), 'ko');
plot(path(:,1), path(:,2), 'r*');
hold off;

end

