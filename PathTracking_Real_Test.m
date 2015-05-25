close all
clear all
format long
delete(timerfindall)

% cleanupObj = onCleanUp(@() pioneer_set_controls(sp,0,0));   

%Constants and Static objects
map = imread('piso5_bw.jpg');
map_bw = imread('piso5_bw_noelevator.bmp');
map_size = size(map);
real_to_map = 26875/map_size(1,1); %ratio betwwen the tower's real dimentsions and the map dimensions, conversion from mm to pixeis
sonar_alert_dist_level_1 = 400; %distance in mm, warning level one used to prevent the robot from turning in that direction
sonar_alert_dist_level_2 = 300; %distance in mm, warning level two minimum distance allowed to an object to prevent collisions
max_robot_error_line2 = 15;
max_robot_error_line3 = 15;
max_robot_error = 30;
max_value_w = 45;
pause_time = .5;
turn_correct = true;

%Obtaining inputs of the robot
% x_init = 130
% y_init = 145
x_init = 50;
y_init = 60;
% theta_initi = input('Robot initial orientation: ');
v_init = 125;
w_init = 0;
time_period = 1;
sp_COM = 'COM11';
mode = 1;
path_points = [];

if mode < 3 && mode > 0
     waypoints = [x_init y_init; 54 100; 56 110; 58 125; 64 140; 130 144; 420 145; 433 170; 433 420; 420 430; 155 430; 140 415; ...
         135 165; 135 155; 130 144; 64 140; 58 125; 56 110; 54 100; x_init y_init];
%         waypoints = [x_init y_init; 54 100; 56 110; 58 125; 64 140; 130 144; 132.5 150; 135 155; 138 165; 140 415; 155 430; 420 430; 433 420; 433 170; 420 145; ...
%          130 144; 64 140; 58 125; 56 110; 54 100; x_init y_init];
%     waypoints = [x_init y_init; 50 100; 45 120; 45 130; 50 140; 130 145;415 145; 435 175; 435 415; 415 430; 160 430; 140 415; 140 175; 130 145; 58 145; 50 120; x_init y_init];
%     waypoints = [x_init y_init; 420 145; 433 170; 433 420; 420 430; 155 430; 140 415; 135 165; 135 155; x_init y_init];
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
theta = fix(atan2(path(2,1)-path(1,1),path(2,2)-path(1,2))*(360/2)/pi);
theta_offset = 90;
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
v_last = v_init;
counter = -1;
reachHalf = false;
corrected_horizontal = false;
last_point_index = 0;
last_point_index_counter = 0;

pioneer_init(sp);
pioneer_set_heading(sp,theta)
pause(3);
pioneer_set_controls(sp,v_real,w_real)

%Timer Set
%timer_obj = timer('TimerFcn', 'MyTimeCallback(sp, v_real, w_real, x, y, map, velocities, path)', 'Period', time_period, 'ExecutionMode', 'fixedSpacing');
%start(timer_obj);

%Main Loop
while point_index ~= path_real_len
    
    %Update velocity and angular velocity
    pause_time = 0.5;
    v_real = v_init;
    odometry_data = pioneer_read_odometry;
    x_odo = odometry_data(1);
    y_odo = odometry_data(2);
    theta_read = fix(odometry_data(3)*360/4096);
    theta_odo = theta_read;
    if theta_odo > 180
        theta_odo = theta_odo - 360;
    elseif theta_odo < -180
        theta_odo = theta_odo + 360;
    else
        theta_odo = theta_odo;
    end
    
    theta = theta_odo;
    x = x_init_real + x_odo;
    y = y_init_real + y_odo;
    rec_moved = [rec_moved;x y theta];
    if point_index < half_path_real_len
        path_interval = path_real(1:half_path_real_len,:); 
        point_index = Find_Nearest_Point_Real(x, y, path_interval, point_index);
    else
        point_index = min(Find_Nearest_Point_Real(x, y, path_real, point_index), path_real_len);
    end
    
    if point_index == path_real_len
        pioneer_close(sp);
        break;
    end
    
    if(point_index == last_point_index)
        last_point_index_counter = last_point_index_counter + 1;
        if (last_point_index_counter == 3)
            point_index = point_index + 1;
            last_point_index_counter = 0;
        end
    end
    
    x_ref = path_real(point_index, 2);
    y_ref = path_real(point_index, 1);
    theta_ref = atan2(1000*(path_real(point_index+1,1) - y_ref), 1000*(path_real(point_index+1,2) - x_ref))*(360/2)/pi;
    rec_ref = [rec_ref;x_ref y_ref theta_ref point_index];
    if point_index == 1
        delta_theta = 0;
    else
        delta_theta = theta_ref - atan2(1000*(y_ref - path_real(point_index-1,1)), 1000*(x_ref - path_real(point_index-1,2)))*(360/2)/pi;
    end
        
    w_ref_sqrt = sqrt((path_real(point_index+1,1) - path_real(point_index, 1))^2 + (path_real(point_index+1,2) - path_real(point_index, 2))^2);
    w_ref = delta_theta * v_real / w_ref_sqrt;
    
    if abs(w_ref) > max_value_w
        w_ref = max_value_w * sign(w_ref);
    end
    
    theta_error = theta_ref - theta;
    if theta_error > 180
        theta_error = theta_error - 360;
    elseif theta_error < -180
        theta_error = theta_error + 360;
    end
    
    errors_world = [x_ref - x; y_ref - y; theta_error];
    
    if((x_init_real - x_ref)^2 + (y_init_real - y_ref)^2) < (4500^2 + 3900^2)
        if abs(errors_world(1)) > 400
            errors_world(1) = 400 * sign(errors_world(1));
        end
        if abs(errors_world(2)) > 400
            errors_world(2) = 400 * sign(errors_world(2));
        end
    else
        if abs(errors_world(1)) > 500
            errors_world(1) = 500 * sign(errors_world(1));
        end
        if abs(errors_world(2)) > 500
            errors_world(2) = 500 * sign(errors_world(2));
        end
    end
    errors_robot = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1] * errors_world;
    
    % Adjustments due to errors of the robot against the reference path
    robot_internal_error = 0.5 * robot_internal_error * scale_error + errors_robot(2) * scale_error;
    
    b = 0.001;
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
       
    %Adjustments due to the sonar information
    optical_data = pioneer_read_sonars;
    
    if((x_init_real - x_ref)^2 + (y_init_real - y_ref)^2) < (4500^2 + 2100^2)
        optical_data = ones(1,8)*5000;
        disp('No sonars!!');
    elseif((x_init_real - x_ref)^2 + (y_init_real - y_ref)^2) < (4500^2 + 2400^2)
        optical_data(:,5:8) = ones(1,4)*5000;
        disp('Only left side!!');
    end
    
    if length(optical_data) == 8
        rec_optical_data = [rec_optical_data;optical_data];
    else
        rec_optical_data = [rec_optical_data; ones(1,8)*5000];
    end
    
    sensors_angles = [90 50 30 10 -10 -30 -50 -90];
    lateral_sensors_status = [0 0 0 0];
    
    adjust_optical = 0;
    danger_level = 0;
    
    %90 degrees sonars check
    if(optical_data(1) <= sonar_alert_dist_level_2)
        lateral_sensors_status(1) = 2;
    elseif(optical_data(1) < sonar_alert_dist_level_1)
        lateral_sensors_status(1) = 1;
    end
    
    if(optical_data(8) <= sonar_alert_dist_level_2)
        lateral_sensors_status(4) = 2;
    elseif(optical_data(8) < sonar_alert_dist_level_1)
        lateral_sensors_status(4) = 1;
    end
    
    %50 degrees sonars check
    if(optical_data(2) <= 200)
        lateral_sensors_status(2) = 2;
        disp('ajuste sonar 50º esquerda');
        adjust_optical = adjust_optical - (135 - abs(sensors_angles(2)))*(200 - optical_data(2));
    elseif(optical_data(2) < 300)
        lateral_sensors_status(2) = 1;
    end
    
    if(optical_data(7) <= 200)
        lateral_sensors_status(3) = 2;
        disp('ajuste sonar 50º direita');
        adjust_optical = adjust_optical + (135 - abs(sensors_angles(7)))*(200 - optical_data(7));
    elseif(optical_data(7) < 300)
        lateral_sensors_status(3) = 1;
    end
    
    %frontal sonars (10 and 30 degrees) check
    for i = 3:6
        if(danger_level ~= 2)
            if(optical_data(i) <= 250)
                danger_level = 2;
            elseif(optical_data(i) < 350)
                danger_level = 1;
            end
        end
        if(optical_data(i) < 500)
                if i < 5
                    adjust_optical = adjust_optical - (135 - abs(sensors_angles(i)))*(500 - optical_data(i));
                else
                    adjust_optical = adjust_optical + (135 - abs(sensors_angles(i)))*(500 - optical_data(i));
                end
        end
    end
    
    adjust_optical = adjust_optical / 4000*scale_error*10;
    
    if abs(adjust_robot_errors2) > 15 * robot_internal_error
        adjust_robot_errors3 = adjust_robot_errors3 * 0.3;
    end
    
    if abs(adjust_optical) > 15 * robot_internal_error
        adjust_robot_errors3 = 0.3 * adjust_robot_errors3;
        adjust_robot_errors2 = 0.3 * adjust_robot_errors2;
    end
    
    adjust_robot_errors = adjust_robot_errors2 + adjust_robot_errors3;
    
    if abs(adjust_robot_errors) > max_robot_error
        adjust_robot_errors = max_robot_error * sign(adjust_robot_errors);
    end
    
    w_real = fix(w_ref + adjust_robot_errors + adjust_optical);
    
    if(w_real > 0 && (lateral_sensors_status(1) == 2 || lateral_sensors_status(1) == 1 || lateral_sensors_status(2) == 1))
        disp('ajuste sonar 90º esquerda');
        w_real = 0;
    elseif(w_real < 0 && ((lateral_sensors_status(4) == 2 || lateral_sensors_status(4) == 1 || lateral_sensors_status(3) == 1)))
        disp('ajuste sonar 90º direita');
        w_real = 0;
    end
    
    if abs(w_real) > max_value_w
        w_real = max_value_w * sign(w_real);
    end
        
    if abs(w_real) > 15
        disp('desaceleraçao')
        v_real = max(75 ,v_last - fix(v_last/16));
        v_last = v_real;
    elseif v_last < v_init && abs(w_real) <= 15;
        disp('aceleraçao')
        v_real = min(v_last + fix(v_last*0.0625),v_init);
        v_last = v_real;
    elseif v_last == v_real && v_real == v_init
        if counter < 0
            counter = 0;
        end
        counter = counter + pause_time;
        if((((x_init_real - x_ref)^2 + (y_init_real - y_ref)^2) > (4500^2 + 2400^2)) && mod(counter, 10) == 0)
            disp('correcçao horizontal')
%            corrected_horizontal = true;
           w_real = w_real - fix(0.044747349*360/(2*pi));
%            v_real = v_init + 25;
        end
    end
    
    if turn_correct
        if w_real > 20
            disp('correcçao rotacional')
            w_real = w_real - 15; 
        elseif w_real < -20
            disp('correcçao rotacional')
            w_real = w_real + 15; 
        end
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
%     if corrected_horizontal
%         pause(pause_time)
%         w_real = w_real + fix(0.044747349*360/(2*pi))*2;
%         corrected_horizontal = false;
%     end
    figure(1);
    subplot(1,2,1); subimage(map_bw); hold on;
    plot(path(:,1), path(:,2)); hold on;
    plot(path(point_index, 1), path(point_index, 2), 'r*'); hold on;
    subplot(1,2,2); plot(velocities_ref(:,2),'b*'); hold on;
    subplot(1,2,2); plot(velocities(:,1),'bo'); hold on;
    subplot(1,2,2); plot(velocities(:,2),'r*'); hold off
    last_point_index = point_index;
    pause(pause_time)
end
% stop(timer_obj);
% delete(timer_obj);
serial_port_stop(sp);

format short;





