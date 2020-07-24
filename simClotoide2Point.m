clear
clc

x = [0 1];
y = [0 1];
cita = [45 20]*pi/180;

dr = 1;
vr = 1;
dt =0.005;

[xr,yr,kr,citar,fir] = CurvaClotoideEntrePuntos(x,y,cita,dr,vr,dt);

plot(xr,yr);
grid
figure(2)
plot(xr,citar*180/pi);
grid