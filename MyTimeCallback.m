function [ ] = MyTimeCallback(serial_port, speed, ang_speed, x, y, map, velocities, ref_path )
%MyTimeCallback - Function that everytime timer is called updates the robot
%and plots shown.

pioneer_set_controls(serial_port, speed, ang_speed);

figure(1);
subplot(1,2,1); subimage(map); hold on;
plot(ref_path(:,1),ref_path(:,2));
plot(x,y,'r*');
subplot(1,2,2); plot(velocities(:,1),'bo'); hold on;
subplot(1,2,2); plot(velocities(:,2),'r*'); hold on;
end

