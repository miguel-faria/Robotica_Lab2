function [ path, x, y ] = Create_Path( map, nav_points, s_tree ,waypoints, mode )
%Create_Path - Generates the path points for the robot to go through the
%given waypoints using the information about the map, the possible
%navigation points and the search tree. The mode parameter chooses between
%the kind of interpolation used (1 - pchip, 2 - spline)

path_points = A_Star_best_path(map, nav_points, s_tree, waypoints);
t = linspace(0,1000,length(path_points(:,1)))';

if mode == 1
    ppx = pchip(t, path_points(:,2));
    ppy = pchip(t, path_points(:,1));
    x = ppval(ppx, linspace(0,1000,10000))';
    y = ppval(ppy, linspace(0,1000,10000))';
        
elseif mode == 2
    ppx = spline(t, path_points(:,2));
    ppy = spline(t, path_points(:,1));
    x = ppval(ppx, linspace(0,1000,10000))';
    y = ppval(ppy, linspace(0,1000,10000))';
    
else
   error('Invalid interpolation mode!!'); 
end

path = [x, y];

image(map);hold on;
plot(y, x, 'r');
end

