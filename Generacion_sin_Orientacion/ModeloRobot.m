function [dx,dy,dcita,dfi] = ModeloRobot(x,y,cita,fi,u1,u2)

dx = cos(cita)*2;
dy = sin(cita)*2;
dcita = tan(fi)*u2;
dfi = u1;
end

