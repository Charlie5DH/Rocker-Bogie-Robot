%cada uno de los puntos planificados P1 y P2 para la trayectoria optima pueden ser
%unidos por una de las 4 curvs basicas.
%estas curvas seran realizables si cada una de las clotoides simetricas por
%las que esta compuesta es realizable

CITA_FIN = 0;                                   %angulo final de direccion del chasis 
CITA_INI = 0;                                   %angulo inicial de direccion del chasis
length = size(path_x);

TA_x = 2*length(2) -1;                        %trayectoria auxiliar
TA_y = 2*length(2) -1;                         

%cita_TD(1) = cita_i;
for i = 1: 1: length(2)-1
    CITA_TD(i) = atan2((path_y(i+1) - path_y(i)),(path_x(i+1) - path_x(i)));
    CITA_TD(i) = rad2deg(CITA_TD(i));
end
CITA_TD(length(2)) = CITA_FIN;

j = 1;
for i = 1: 1: 2*length(2) -1
    if(mod(i,2) ~= 0)
        TA_x(i) = path_x((i+1)/2);
        TA_y(i) = path_y((i+1)/2);
    end
    if(mod(i,2) == 0)
       j = j+1;
       TA_x(i) = path_x(j) + ( path_x(j-1) - path_x(j) )/2;
       TA_y(i) = path_y(j) + ( path_y(j-1) - path_y(j) )/2;
    end
end

%cita_TA(1) = cita_i;
for j = 1: 1: 2*length(2) -2
    CITA_TA(j) = atan2((TA_y(j+1) - TA_y(j)),(TA_x(j+1) - TA_x(j)));
    CITA_TA(j) = rad2deg(CITA_TA(j));
end
CITA_TA(2*length(2) -1) = CITA_FIN;

clear C;
spy(mapMatrix);
hold on;
p = plot(TA_x,TA_y,'k');
p.LineWidth = 2;
grid on;
xticks(0:1:a_terreno/Re);
yticks(0:1:a_terreno/Re);
axis([0 a_terreno/Re 0 a_terreno/Re]);


%D = rad2deg(R)





