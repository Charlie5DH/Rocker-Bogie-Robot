function[dx,dy,dcita,dfi] = ModeloTriciclo(x,y,cita,fi,u1,u2)
dx = cos(cita)*u2;
dy = sin(cita)*u2;
dcita = tan(fi)*u2;
dfi = u1;
end