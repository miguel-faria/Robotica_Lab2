close all;
clear all;

format long;

mapa = imread('piso5_orig.bmp');
mapa_bw = imread('piso5_bw_noelevator.bmp');
delta_time = 2;
path_points = [];
% waypoints = [35 50;35 150;130 150;433 147;433 433;147 433;130 150;35 150;35 50];
mode = input('Choose the mode: \n1 - interpolation with pchip\n2 - interpolation with spline\n3 - use static path\nMode: ');
if mode < 3 && mode > 0
    disp('Give me the waypoints');
    n_waypoints = input('How many waypoints do you want: ');
    for i = 1:n_waypoints
       fprintf('Waypoint %d:\n', i);
       x_coord = input('X Coordinate: '); 
       y_coord = input('Y Coordinate: ');
       waypoints(i, :) = [x_coord y_coord];
    end
    
    [nav_points, s_tree] = Get_Navigatable_Points(mapa_bw);
    path_points = A_Star_best_path(map, nav_points, s_tree, waypoints);
end
   
[path,x,y] = Create_Path(mapa_bw, path_points, mode);
pause;

robot_path = Path_Tracking_Virtual(path, 2, delta_time, mapa);

format short