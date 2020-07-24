function [dx,dy,dcita,dfi] = modeloAckermanFWD(x,y,cita,fi,u1,u2,L)
dx = cos(fi)*cos(cita)*u2;
dy = cos(fi)*sin(cita)*u2;
dcita = sin(fi)*u2/L;
dfi = u1;
end

