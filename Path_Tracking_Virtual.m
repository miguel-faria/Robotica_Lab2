function [ ref_path ] = Path_Tracking_Virtual( path, speed, delta_time, map)

x = path(1,1);
y = path(1,2);
index = 1;
theta = atan2(path(2,2)-path(1,2),path(2,1)-path(1,1));
w = 0;
point_index = 1;
noise = speed/100;
path_len = length(path);
image(map); hold;

while point_index ~= path_len
    
%     first = point_index - 10;
%     last = point_index + 10;
%     if first < 0
%         path_interval = path(1:(last + (point_index - 1)),:);
%     else if last > path_len
%             path_interval = path((first - (path_len - point_index)):path_len,:);
%         else
%            path_interval = path(first:last,:);
%         end
%     end
    if point_index < floor(path_len/2)
        path_interval = path(1:floor(path_len/2),:);
        point_index = Find_Nearest_Point(x, y, path_interval);
    else
        path_interval = path(floor(path_len/2):path_len,:);
        point_index = min((Find_Nearest_Point(x, y, path_interval) + floor(path_len/2)), path_len);
    end
    
    if point_index == path_len
        break;
    end
    
    x_ref = path(point_index, 1);
    y_ref = path(point_index, 2);
    ref_path(index,:) = [x_ref; y_ref];
    index = index + 1;
    theta_ref = atan2((path(point_index+1,2)) - y_ref, (path(point_index+1,1) - x_ref));
    if point_index == 1
        delta_theta = 0;
    else
        delta_theta = theta_ref - atan2((y_ref - path(point_index-1,2)), (x_ref - path(point_index-1,1)));
    end
    w_ref = delta_theta * speed / sqrt((path(point_index+1,1) - x_ref)^2 + (path(point_index+1,2) - y_ref)^2);
    
    errors_world = [x_ref - x; y_ref - y; theta_ref - theta];
    errors_robot = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1] * errors_world;
    
    b = 0.05;
    eps = 0.01;
    
    k2 = b*abs(speed);
    k3 = 2*eps*sqrt(w_ref^2 + b*speed^2);
    
    adjust = errors_robot(2)*k2 + errors_robot(3)*k3;
    if abs(adjust) > 0.5
        adjust = 0.5*sign(adjust);
    end
    
    w = w_ref + adjust;
        
    theta = theta + w*delta_time + 0.001*randn(1);
    
    x = x + speed*cos(theta)*delta_time + noise*speed/50*randn(1);
    y = y + speed*sin(theta)*delta_time + noise*randn(1);
    
    plot(path(:,1),path(:,2));
    plot(x,y,'o');
%     plot([x;x+3*cos(theta)],[y;y+3*sin(theta)]);
    plot(x_ref,y_ref,'marker','o','color','r');
    fprintf('X: %2.4f\tY: %2.4f\nX_REF: %2.4f\tY_REF: %2.4f\n\n', x, y, x_ref, y_ref);
    pause(0.05);
end

hold off;
end

