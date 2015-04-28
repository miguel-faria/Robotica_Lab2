close all;
clear all;

format long;

psi = [(pi)*ones(1,150) pi:pi/99:3*pi/2 (3*pi/2)*ones(1,140) 3*pi/2:pi/99:2*pi (2*pi)*ones(1,150)...
        (2*pi)*ones(1,30) (2*pi)*ones(1,540) 2*pi:-pi/139:3*pi/2 (3*pi/2)*ones(1,480) 3*pi/2:-pi/139:pi...
        (pi)*ones(1,500) pi:-pi/139:pi/2 (pi/2)*ones(1,480) pi/2:pi/139:pi (pi)*ones(1,130)...
        pi:-pi/99:pi/2 (pi/2)*ones(1,140) pi/2:-pi/99:0 (0)*ones(1,130)];
speed = 0.5;
x1 = 129;
y1 = 45;

path = Create_Reference_Path(speed,psi,x1,y1);
x = path(1:length(path),1);
y = path(1:length(path),2);

len = length(psi);
i1 = imread('piso5_orig.bmp');
image(i1);hold;
for i=1:len
   plot(x(i),y(i));
   %pause(0.00000000001);
end

format short;