function[di,dj]=DireccionIJ(cita)
% Determina la direccion de las componentes i,j.
% Devuelve:
% di,dj: direccion de las componentesi,j respectivamente. 1--> aumenta en
% valor, -1--> disminuye.
% Recibe:
% cita: angulo de orientacion.

% ajustando cita al rango 0--2pi  
cita = AngRango0a2pi(cita);

if( 3*pi/2 < cita || cita < pi/2)
    di = 1;
end

if( pi/2 < cita && cita  < 3*pi/2)
    di = -1;
end

if(cita == 3*pi/2 || cita == pi/2)
    di = 0;
end

if( 0 < cita && cita  < pi)
    dj = 1;
end

if( pi< cita && cita  < 2*pi)
    dj = -1;
end

if(cita == 0 || cita == pi || cita == 2*pi)
    dj = 0;
end

