function [ output_args ] = Path_Tracking_Virtual( path, speed, delta_time )

x = path(1,1);
y = path(1,2);
theta = atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
w = 0;
point_index = 1;
noise = speed/100;
path_len = length(path);

while point_index ~= path_len

    point_index = Find_Nearest_Point(x, y, path);
    if point_index == path_len
        break;
    end
    
    x_ref = path(point_index, 1);
    y_ref = path(point_index, 2);
    theta_ref = atan2(path(point_index+1,2) - y_ref, path(point_index+1,1) - x_ref);
    if point_index == 0
        delta_theta = 0;
    else
       delta_theta = theta_ref - atan2(y_ref - path(point_index-1,2), x_ref - path(point_index-1,1)); 
    end
    w_ref = delta_theta * speed / sqrt((path(point_index+1,1) - x_ref)^2 + (path(point_index+1,2) - y_ref)^2);
    
    errors_world = [x_ref - x; y_ref - y; theta_ref - theta];
    errors_robot = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1] * errors_world;
    
    adjust = errors_robot(3)*.2*sqrt(w_ref^2+0.005*speed^2) + errors_robot(2)*.5;
    if abs(adjust) > 0.3
        adjust = 0.3*sign(adjust);
    end
    
    w = w_ref + adjust;
end
end

