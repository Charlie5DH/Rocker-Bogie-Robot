function [dx,dy,dcita] = ModeloDiferencial(cita,u1,u2)
dcita = u1;
dx = cos(cita)*u2;
dy = sin(cita)*u2;

end

