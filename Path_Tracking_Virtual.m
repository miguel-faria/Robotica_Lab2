function [ robot_path ] = Path_Tracking_Virtual( path, speed, delta_time, map)
%Path_Tracking_Virtual - virtual simulation for the robot following a given
%path with a certain speed and interval between communications. This
%function also prints on the map the robot current position and the
%position where the robot should be

x = path(1,1);
y = path(1,2);
index = 1;
theta = atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
w = 0;
point_index = 1;
noise = speed/75;
path_len = length(path);
half_path_len = fix(path_len/2);

while point_index ~= path_len
        
    if point_index < half_path_len
        path_interval = path(1:half_path_len,:);
        point_index = Find_Nearest_Point(x, y, path_interval);
    else
        path_interval = path(half_path_len:path_len,:);
        point_index = min((Find_Nearest_Point(x, y, path_interval) + half_path_len), path_len);
    end
    
    if point_index == path_len
        break;
    end
    
    x_ref = path(point_index, 1);
    y_ref = path(point_index, 2);
    theta_ref = atan2((path(point_index+1,2)) - y_ref, (path(point_index+1,1) - x_ref));
    if point_index == 1
        delta_theta = 0;
    else
        delta_theta = theta_ref - atan2((y_ref - path(point_index-1,2)), (x_ref - path(point_index-1,1)));
    end
    
    w_ref_sqrt = sqrt((path(point_index+1,1) - x_ref)^2 + (path(point_index+1,2) - y_ref)^2);
    w_ref = delta_theta * speed / w_ref_sqrt;
    
    if abs(w_ref) > 0.5
        w_ref = 0.5*sign(w_ref);
    end
    
    errors_world = [x_ref - x; y_ref - y; theta_ref - theta];
    errors_robot = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1] * errors_world;
    
    b = 0.05;
    qsi = 0.01;
    
    k2 = b*abs(speed);
    k3 = 2*qsi*sqrt(w_ref^2 + b*speed^2);
    
    adjust = errors_robot(2)*k2 + errors_robot(3)*k3;
    if abs(adjust) > 0.5
        adjust = 0.5*sign(adjust);
    end
    
    w = w_ref + adjust;
        
    random_val = randi([-100000,100000],1,3)/500000;
    
    theta = theta + w*delta_time + 0.001*random_val(3);
    
    x = x + speed*cos(theta)*delta_time + noise*speed/50*random_val(1);
    y = y + speed*sin(theta)*delta_time + noise*random_val(2);
    
    robot_path(index,:) = [x y];
    velocities_path(index,:) = [speed w];
    ref_velocities(index,:) = [speed w_ref];
    index = index + 1;
    
    
    figure(1);
    subplot(1,2,1); subimage(map); hold on;
    plot(path(:,1),path(:,2));
    plot(x,y,'r*');
%     plot(x_ref,y_ref,'b*');
    fprintf('X: %2.4f\tY: %2.4f\nX_REF: %2.4f\tY_REF: %2.4f\n\n', x, y, x_ref, y_ref); %hold off;
%     figure(2); hold on;
    subplot(1,2,2); plot(velocities_path(:,2),'r*'); hold on;
    subplot(1,2,2); plot(ref_velocities(:,2), 'b*'); hold off;
%     pause(0.0005);
end

hold off;
end

