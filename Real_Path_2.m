close all;
clear all;

format long;

psi = [(2*pi)*ones(1,140) 2*pi:pi/139:5*pi/2 (pi/2)*ones(1,60) pi/2:pi/139:pi (pi)*ones(1,140) pi:pi/139:3*pi/2 (3*pi/2)*ones(1,60) 3*pi/2:pi/139:2*pi];
speed = 0.5;
x1 = 50;
y1 = 110;

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