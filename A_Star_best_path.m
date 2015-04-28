function [ path ] = A_Star_best_path( map, nav_points, waypoints )
%A_Star_best_path - A star search for the best path in a given map that
%passes through the given waypoints. To note that the waypoints must be at
%least 2.

path = [];
n_points = size(waypoints);

if n_points(1,1) < 2
    error('There must be at least two waypoints: one starting point and one destination point.');
end

for i = 1:n_points
    
    if i == n_points
        break;
    end
    
    start_point = waypoints(i,:);
    end_point = waypoints(i+1,:);
    
    path = [path; A_star_search(start_point, end_point, nav_points)];
end

image(map); hold;
plot(path(:,1), path(:,2));

end

