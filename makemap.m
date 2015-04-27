% i1 = imread('piso5_.bmp');
% imshow(i1); hold;

x1 = [43.2500 61.2500 154.2500 413.7500 440.7500 444.7500];
y1 = [55.0000 140.5000 154.0000 152.5000 157.0000 434.5000];
t1 = 43.2:0.01:444.75;
p1 = pchip(x1,y1,t1);
plot(t1,p1,'r')

x2 = [ 444.7500 430.9266 161.0037 154.4202 146.7394 60.0569 45.7927];
y2 = [ 434.500 441.0450 442.1422 427.8780 160.6633 140.3991 54.8138];
t2 = 45.5:0.01:446;
p2 = pchip(x2,y2,t2);
plot(t2,p2,'r');