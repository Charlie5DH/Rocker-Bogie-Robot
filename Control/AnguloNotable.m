function[anguloGiro]=AnguloNotable(cita)
%busca el angulo notable de giro(0,45,90,135,180,225,270,315,360) con que debe
%comenzar o terminar la manieobra de giro.
% para los angulos sobre los ejes x,y la diferencia entre cita y el
% respectivo angulo debe ser menor que 20 para que pueda ser realizable la
% curva inicial o final.

anguloGiro = 0;
if(abs(cita - 0) < 20*pi/180)
    anguloGiro = 0;
end
if(abs(pi/4 - cita) <= 25*pi/180)
    anguloGiro = pi/4;
end
if(abs(pi/2 - cita) < 20*pi/180)
    anguloGiro = pi/2;
end
if(abs(3*pi/4 - cita) <= 25*pi/180)
    anguloGiro = 3*pi/4;
end
if(abs(pi- cita) < 20*pi/180)
    anguloGiro = pi;
end
if(abs(5*pi/4 - cita) <= 25*pi/180)
    anguloGiro = 5*pi/4;
end
if(abs(3*pi/2 - cita) < 20*pi/180)
    anguloGiro = 3*pi/2;
end
if(abs(7*pi/4 - cita) <= 25*pi/180)
    anguloGiro = 7*pi/4;
end
if(abs(2*pi - cita)  < 20*pi/180)
    anguloGiro = 2*pi;
end