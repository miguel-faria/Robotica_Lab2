function [ path, x, y ] = Create_Path( map, path_points, mode )
%Create_Path - Generates the path points for the robot to go through the
%given waypoints using the information about the map, the possible
%navigation points and the search tree. The mode parameter chooses between
%the kind of interpolation used (1 - pchip, 2 - spline, 3 - use static path, 4 - use static path 2)

if mode == 1
    t = linspace(0,1000,length(path_points(:,1)))';
    ppx = pchip(t, path_points(:,2));
    ppy = pchip(t, path_points(:,1));
    x = ppval(ppx, linspace(0,1000,1000))';
    y = ppval(ppy, linspace(0,1000,1000))';
    path = [y, x];

    image(map);hold on;
    plot(y, x, 'r');
    
elseif mode == 2
    t = linspace(0,1000,length(path_points(:,1)))';
    ppx = spline(t, path_points(:,2));
    ppy = spline(t, path_points(:,1));
    x = ppval(ppx, linspace(0,1000,1000))';
    y = ppval(ppy, linspace(0,1000,1000))';
    path = [y, x];

    image(map);hold on;
    plot(y, x, 'r');
    
elseif mode == 3
    psi = [(pi)*ones(1,150) pi:pi/99:3*pi/2 (3*pi/2)*ones(1,140) 3*pi/2:pi/99:2*pi (2*pi)*ones(1,150)...
        (2*pi)*ones(1,30) (2*pi)*ones(1,540) 2*pi:-pi/139:3*pi/2 (3*pi/2)*ones(1,480) 3*pi/2:-pi/139:pi...
        (pi)*ones(1,500) pi:-pi/139:pi/2 (pi/2)*ones(1,480) pi/2:pi/139:pi (pi)*ones(1,130)...
        pi:-pi/99:pi/2 (pi/2)*ones(1,140) pi/2:-pi/99:0 (0)*ones(1,130)];
    speed = 0.5;
    x1 = 128;
    y1 = 47;

    path = Create_Reference_Path(speed,psi,x1,y1);
    x = path(1:length(path),1);
    y = path(1:length(path),2);
   
    image(map);hold on;
    plot(x, y, 'r');
    
elseif mode == 4
    psi = [(2*pi)*ones(1,140) 2*pi:pi/139:5*pi/2 (pi/2)*ones(1,60) pi/2:pi/139:pi (pi)*ones(1,140) pi:pi/139:3*pi/2 (3*pi/2)*ones(1,60) 3*pi/2:pi/139:2*pi];
    speed = 0.5;
    x1 = 50;
    y1 = 110;

    path = Create_Reference_Path(speed,psi,x1,y1);
    x = path(1:length(path),1);
    y = path(1:length(path),2);
   
    image(map);hold on;
    plot(x, y, 'r');
    
else
   error('Invalid interpolation mode!!'); 
end

end

