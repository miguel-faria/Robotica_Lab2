close all;
clear all;
format long;
delete(timerfindall);

%Constants and Static objects
map = imread('piso5_orig.bmp');
map_bw = imread('piso5_bw_noelevator.bmp');
map_size = size(map);
real_to_map = 26875/map_size(1,1); %ratio betwwen the tower's real dimentsions and the map dimensions, conversion from mm to pixeis
sonar_alert_dist_level_1 = 250; %distance in mm, warning level one used to prevent the robot from turning in that direction
sonar_alert_dist_level_2 = 150; %distance in mm, warning level two minimum distance allowed to an object to prevent collisions
max_robot_error_line2 = 15;
max_robot_error_line3 = 15;
max_value_w = 45;
pause_time = 1;

%Obtaining inputs of the robot
x_init = input('Robot X coord starting position on map: ');
y_init = input('Robot Y coord starting position on map: ');
% theta_initi = input('Robot initial orientation: ');
v_init = input('Robot starting linear speed: ');
w_init = input('Robot starting angular velocity: ');
time_period = input('Timer period: ');
sp_COM = input('COM port robot is plugged into: ');
mode = input('Choose the mode: \n1 - interpolation with pchip\n2 - interpolation with spline\n3 - use static path\n4 - use static path 2\nMode: ');
path_points = [];

if mode < 3 && mode > 0
    disp('Give me the waypoints');
    n_waypoints = input('How many waypoints do you want: ');
    for i = 1:n_waypoints
        fprintf('Waypoint %d:\n', i);
        x_coord = input('X Coordinate: ');
        y_coord = input('Y Coordinate: ');
        waypoints(i, :) = [x_coord y_coord];
    end
    
    [nav_points, s_tree] = Get_Navigatable_Points(map_bw);
    path_points = A_Star_best_path(map_bw, nav_points, s_tree, waypoints);
end


if isempty(path_points)
    [path,x_res,y_res] = Create_Path(map_bw, path_points, mode);
    for i = 1:length(path)
        path_real(i,:) = [(path(i,1))*real_to_map (path(i,2))*real_to_map];
    end
else
    for i = 1:length(path_points)
        path_points_real(i,:) = [(path_points(i,1))*real_to_map (path_points(i,2))*real_to_map];
    end
    [path_real,x_res,y_res] = Create_Path(map_bw, path_points_real, mode);
    [path,x_res,y_res] = Create_Path(map_bw, path_points, mode);
end

clear x_res;
clear y_res;
clear s_tree;
clear nav_points;

%Variable Declaration
index = 1;
v_real = v_init;
w_real = w_init;
w_map = 0;
x_map = x_init;
y_map = y_init;
x_init_real = x_init * real_to_map;
y_init_real = y_init * real_to_map;
theta = fix(atan2(path_real(2,2)-path_real(1,2),path_real(2,1)-path_real(1,1))*(360/2)/pi);
theta_map = atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
velocities(index, :) = [v_init w_init];
index = index + 1;
sp = serial_port_start(sp_COM);
point_index = 1;
path_real_len = length(path_real);
half_path_real_len = fix(path_real_len/2);
robot_internal_error = 0;
scale_error = v_init/100;
odometry_data = [];
optical_data = [];
rec_optical_data = [];
rec_moved = [];
rec_ref = [];
t = 1;

pioneer_init(sp)
pioneer_set_heading(sp,theta)
pause(3);
pioneer_set_controls(sp,v_real,w_real)

%Timer Set
%timer_obj = timer('TimerFcn', 'MyTimeCallback(sp, v_real, w_real, x, y, map, velocities, path)', 'Period', time_period, 'ExecutionMode', 'fixedSpacing');
%start(timer_obj);

%Main Loop
while point_index ~= path_real_len
    
    %Update velocity and angular velocity
    v_real = v_init;
    odometry_data = pioneer_read_odometry;
    x_odo = odometry_data(1);
    y_odo = odometry_data(2);
    theta_odo = fix(odometry_data(3)*360/4096);
    if theta_odo > 180
        theta_odo = theta_odo - 360;
    else
        theta_odo = theta_odo;
    end
    
    theta = theta_odo;
    x = x_init_real + x_odo;
    y = y_init_real + y_odo;
    rec_moved = [rec_moved;x y theta];
    if point_index < half_path_real_len
        path_interval = path_real(1:half_path_real_len,:); 
        point_index = Find_Nearest_Point(x, y, path_interval)
    else
        path_interval = path_real(half_path_real_len:path_real_len,:);
        point_index = min((Find_Nearest_Point(x, y, path_interval) + half_path_real_len), path_real_len)
    end
    
    if point_index == path_real_len
        pioneer_close(sp);
        break;
    end
    
    x_ref = path_real(point_index, 1);
    y_ref = path_real(point_index, 2);
    theta_ref = atan2(1000*(path_real(point_index+1,2)) - y_ref, 1000*(path_real(point_index+1,1) - x_ref))*(360/2)/pi;
    rec_ref = [rec_ref;x_ref y_ref theta_ref];
    if point_index == 1
        delta_theta = 0;
    else
        delta_theta = theta_ref - atan2(1000*(y_ref - path_real(point_index-1,2)), 1000*(x_ref - path_real(point_index-1,1)))*(360/2)/pi;
    end
    
    w_ref_sqrt = sqrt((path_real(point_index+1,1) - path_real(point_index, 1))^2 + (path_real(point_index+1,2) - path_real(point_index, 2))^2);
    w_ref = delta_theta * v_real / w_ref_sqrt;
    
    if abs(w_ref) > 45
        w_ref = 45 * sign(w_ref);
    end
    
    theta_error = theta_ref - theta;
    if theta_error > 180
        theta_error = theta_error - 360;
    elseif theta_error < -180
        theta_error = theta_error + 360;
    end
    
    errors_world = [x_ref - x; y_ref - y; theta_error];
    errors_robot = [cos(theta_ref) sin(theta_ref) 0; -sin(theta_ref) cos(theta_ref) 0; 0 0 1] * errors_world;
    
    % Adjustments due to errors of the robot against the reference path
    robot_internal_error = 0.5 * robot_internal_error * scale_error + errors_robot(2) * scale_error;
    
    b = 0.025;
    qsi = 0.9;
    
    k2 = b*abs(v_real)*robot_internal_error;
    k3 = 2*qsi*sqrt(w_ref^2 + b*v_real^2)*scale_error;
    
    adjust_robot_errors2 = errors_robot(2)*k2;
    adjust_robot_errors3 = errors_robot(3)*k3;
    
    if abs(adjust_robot_errors2) > max_robot_error_line2
        adjust_robot_errors2 = max_robot_error_line2*sign(adjust_robot_errors2);
    end
    if abs(adjust_robot_errors3) > max_robot_error_line3
        adjust_robot_errors3 = max_robot_error_line3*sign(adjust_robot_errors3);
    end
    
    adjust_robot_errors = adjust_robot_errors2 + adjust_robot_errors3;
    
    %Adjustments due to the optical information
    optical_data = pioneer_read_sonars;
    if length(optical_data) == 8
        rec_optical_data = [rec_optical_data;optical_data];
    else
        rec_optical_data = [rec_optical_data; ones(1,8)*5000];
    end
    
    sensors_angles = [90 50 30 10 -10 -30 -50 -90];
    lateral_sensors_status = [0 0 0 0];
    
    adjust_optical = 0;
    danger_level = 0;
    
%     %90 degrees sonars check
%     if(optical_data(1) <= sonar_alert_dist_level_2)
%         lateral_sensors_status(1) = 2;
%     elseif(optical_data(1) < sonar_alert_dist_level_1)
%         lateral_sensors_status(1) = 1;
%     end
%     
%     if(optical_data(8) <= sonar_alert_dist_level_2)
%         lateral_sensors_status(4) = 2;
%     elseif(optical_data(8) < sonar_alert_dist_level_1)
%         lateral_sensors_status(4) = 1;
%     end
%     
%     %50 degrees sonars check
%     if(optical_data(2) <= 200)
%         lateral_sensors_status(2) = 2;
%         adjust_optical = adjust_optical - (135 - abs(sensors_angles(2)))*(sonar_alert_dist_level_1 - optical_data(2));
%     elseif(optical_data(2) < 300)
%         lateral_sensors_status(2) = 1;
%     end
%     
%     if(optical_data(7) <= 200)
%         lateral_sensors_status(3) = 2;
%         adjust_optical = adjust_optical + (135 - abs(sensors_angles(2)))*(sonar_alert_dist_level_1 - optical_data(2));
%     elseif(optical_data(7) < 300)
%         lateral_sensors_status(3) = 1;
%     end
%     
%     %frontal sonars (10 and 30 degrees) check
%     for i = 3:6
%         if(danger_level ~= 2)
%             if(optical_data(i) <= 250)
%                 danger_level = 2;
%             elseif(optical_data(i) < 350)
%                 danger_level = 1;
%             end
%         end
%         if(optical_data(i) < 350)
%             if i < 5
%                 adjust_optical = adjust_optical - (135 - abs(sensors_angles(2)))*(sonar_alert_dist_level_1 - optical_data(i));
%             else
%                 adjust_optical = adjust_optical + (135 - abs(sensors_angles(2)))*(sonar_alert_dist_level_1 - optical_data(i));
%             end
%         end
%         
%     end
%     
%     adjust_optical = adjust_optical / 4000*scale_error*10;
    
    w_real = fix(w_ref + adjust_robot_errors + adjust_optical);
    
    if(w_real > 0 && (lateral_sensors_status(1) == 2 || lateral_sensors_status(1) == 1 || lateral_sensors_status(2) == 1))
        w_real = 0;
    elseif(w_real < 0 && ((lateral_sensors_status(4) == 2 || lateral_sensors_status(4) == 1 || lateral_sensors_status(3) == 1)))
        w_real = 0;
    end
    
    if abs(w_real) > max_value_w
        w_real = max_value_w * sign(w_real);
    end
        
    if danger_level == 2
        v_real = fix(v_init/4);
    elseif danger_level == 1;
        v_real = fix(v_init/2);
    end
        
    velocities(index, :) = [v_real w_real];
    velocities_ref(index, :) = [v_real w_ref];
    index = index + 1;
    pioneer_set_controls(sp, v_real, w_real)
    figure(1);
    subplot(1,2,1); subimage(map); hold on;
    plot(path(:,1), path(:,2));
    plot(path(point_index, 1), path(point_index, 2), 'r*');
    subplot(1,2,2); plot(velocities_ref(:,2),'b*'); hold on;
    %subplot(1,2,2); plot(velocities(:,1),'bo'); hold on;
    subplot(1,2,2); plot(velocities(:,2),'r*');
%     pause(pause_time)
end
% stop(timer_obj);
% delete(timer_obj);
serial_port_stop(sp);

format short;





