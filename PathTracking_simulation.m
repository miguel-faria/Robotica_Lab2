close all;
clear all;

format long;

mapa = imread('piso5_orig.bmp');
mapa_bw = imread('piso5_bw_noelevator.bmp');
delta_time = 2;
nav_points = [];
s_tree = [];
waypoints = [50 50;32 130;125 145;433 145;433 431;145 431;125 145;32 130;50 50];
mode = input('Choose the mode: \n1 - interpolation with pchip\n2 - interpolation with spline\n3 - use static path\nMode: ');
% if mode < 3 && mode > 0
%     disp('Give me the waypoints');
%     n_waypoints = input('How many waypoints do you want: ');
%     for i = 1:n_waypoints
%        fprintf('Waypoint %d:\n', i);
%        x_coord = input('X Coordinate: '); 
%        y_coord = input('Y Coordinate: ');
%        waypoints(i, :) = [x_coord y_coord];
%     end
    [nav_points, s_tree] = Get_Navigatable_Points(mapa_bw);
% end
    
[path,x,y] = Create_Path(mapa_bw, nav_points, s_tree, waypoints, mode);
pause;

robot_path = Path_Tracking_Virtual(path, 2, delta_time, mapa);

format short