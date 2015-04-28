close all;
clear all;

format long;

mapa = imread('piso5_orig.bmp');
psi = [(pi)*ones(1,150) pi:pi/99:3*pi/2 (3*pi/2)*ones(1,140) 3*pi/2:pi/99:2*pi (2*pi)*ones(1,150)...
        (2*pi)*ones(1,30) (2*pi)*ones(1,540) 2*pi:-pi/139:3*pi/2 (3*pi/2)*ones(1,480) 3*pi/2:-pi/139:pi...
        (pi)*ones(1,500) pi:-pi/139:pi/2 (pi/2)*ones(1,480) pi/2:pi/139:pi (pi)*ones(1,130)...
        pi:-pi/99:pi/2 (pi/2)*ones(1,140) pi/2:-pi/99:0 (0)*ones(1,130)];
speed = 0.5;
x1 = 135;
y1 = 55;
delta_time = 2.5;

path = Create_Reference_Path(speed,psi,x1,y1);

ref_path = Path_Tracking_Virtual(path, 1.5, delta_time, mapa);

format short